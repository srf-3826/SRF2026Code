// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * MotionEstimator
 *
 * Computes linear velocity, linear acceleration, angular velocity,
 * and angular acceleration using odometry data and a centered
 * 5‑sample Savitzky–Golay filter.
 * (A centered filter requires a delay before calculating so that both past and
 * future samples can be known. For a 5 sample filter, these are the 2 older adjacent samples and the 
 * 2 later adjacent samples, relative to the "current" middle sample. A causal filter uses only past 
 * data samples, and the coefficients are different between the two approaches).
 * Designed primarily for logging max performance during real world driving - giving sort of 
 * continuously sampled parameters that could be substituted for SysID, provided the driver 
 * has "pushed the robot's limits" during the session.
 */
public class MotionEstimator {

    // Enable/disable all computation (for competition mode)
    private boolean m_enabled = true;

    // Last heading angles
    private double m_lastRawHeadingOdom = 0.0;
    private double m_lastRawHeadingGyro = 0.0;

    // History buffers (newest at index 0)
    private final double[] m_xHist = new double[5];             // Odometry x history
    private final double[] m_yHist = new double[5];             // Odometry y history
    private final double[] m_thetaHistOdom = new double[5];     // Odometry heading history
    private final double[] m_thetaHistGyro = new double[5];     // Gyro heading history

    private final double[] m_tHist = new double[5];     // Loop time history - SG filters want fixed sample periods.
                                                        // FRC Loop times vary slightly - so retain last 5 loop times
                                                        // and use the average.
    private final int[] m_idx = new int[5];

    private boolean m_initialized = false;              // Delay all calculations until a full 5 sample
                                                        // history has accumulated
    private double  m_lastTime = Timer.getFPGATimestamp();
    private int     m_newestIndex = 0;
    private int     m_calcIndex = 0;

    // Smoothed outputs
    private double m_vel = 0;
    private double m_accel = 0;
    private double m_jerk = 0;
    private double m_angVelOdom = 0;
    private double m_angAccelOdom = 0;
    private double m_angJerkOdom = 0;
    private double m_angVelGyro = 0;
    private double m_angAccelGyro = 0;
    private double m_angJerkGyro = 0;

    // Max tracking
    private double m_maxVel = 0;
    private double m_maxAccel = 0;
    private double m_maxJerk = 0;
    private double m_maxAngVelOdom = 0;
    private double m_maxAngAccelOdom = 0;
    private double m_maxAngJerkOdom = 0;
    private double m_maxAngVelGyro = 0;
    private double m_maxAngAccelGyro = 0;
    private double m_maxAngJerkGyro = 0;

    public MotionEstimator() {}

    /** Enable or disable computation */
    public void setEnabled(boolean enable) {
        this.m_enabled = enable;
    }

    /** Update estimator with the latest odometry pose */
    public void update(Pose2d pose, Rotation2d gyroYaw2d) {
        double now = Timer.getFPGATimestamp();
        double dt = now - m_lastTime;           // set dt on entry to the latest loop time - 
        m_lastTime = now;                       // later (after storing the latest loop time) dt will 
                                                // be set to the average of the last 5 loop times.
         if (!m_enabled || dt <= 0.0) {
            return;                             // If not enabled, quit now
        }

        double x = pose.getX();
        double y = pose.getY();
        double rawThetaOdom = pose.getRotation().getRadians();
        double rawThetaGyro = gyroYaw2d.getRadians();
        int lastSampleIndex = (m_newestIndex -1 +5) % 5;
        double thetaOdom = unwrapHeading(m_thetaHistOdom[lastSampleIndex], m_lastRawHeadingOdom, rawThetaOdom);
        double thetaGyro = unwrapHeading(m_thetaHistGyro[lastSampleIndex], m_lastRawHeadingGyro, rawThetaGyro);
        m_lastRawHeadingOdom = rawThetaOdom;
        m_lastRawHeadingGyro = rawThetaGyro;

        if (!m_initialized) {
            // While not yet initialized, fill histories (all 4 data being filtered, plus
            // dt) with new samples . m_newestInddex always points to the array location
            // that previously held the oldest data.
            m_xHist[m_newestIndex] = x;
            m_yHist[m_newestIndex] = y;
            m_thetaHistOdom[m_newestIndex] = thetaOdom;
            m_thetaHistGyro[m_newestIndex] = thetaGyro;
            m_tHist[m_newestIndex] = dt;

            // After storing the history data, increment the m_newestIndex,
            m_newestIndex += 1;

            // When m_newestIndex reaches 4 then the arrays are almost fully populated. 
            // Set the m_initialized flag to true, and the final, most recent samples will be stored 
            // at hist[4] the next time update() is called. Preset m_calcIndex
            // to the middle of the array.
            if (m_newestIndex == 4) {
                m_calcIndex = 2;            // Will be Midpoint (center sample) of sample history array
                                            // when update is next called.
                m_initialized = true;
            }
            return;
        }

        // Can't get here unless both enabled and initialized, so proceed to 
        // store the newest samples
        m_xHist[m_newestIndex] = x;
        m_yHist[m_newestIndex] = y;
        m_thetaHistOdom[m_newestIndex] = thetaOdom;
        m_thetaHistGyro[m_newestIndex] = thetaGyro;
        m_tHist[m_newestIndex] = dt;

        // Now set dt to the AVERAGE of the last 5 dt samples
        // Expected variations of dt within FRC should be small, but are not zero. 
        // SG really works best when the sample period is a constant value, but we 
        // compromize here by averaging the actual dt periods over the sample 
        // history window. 
        dt = (m_tHist[0] + m_tHist[1] + m_tHist[2] + m_tHist[3] +  m_tHist[4]) /  5.0;

        // Build index map: oldest -> newest relative to the middle sample
        m_idx[0] = (m_calcIndex + 3) % 5;  // mid - 2 (oldest)
        m_idx[1] = (m_calcIndex + 4) % 5;  // mid - 1
        m_idx[2] = m_calcIndex;            // mid
        m_idx[3] = (m_calcIndex + 1) % 5;  // mid + 1
        m_idx[4] = (m_calcIndex + 2) % 5;  // mid + 2 (newest)

        // Now advance m_newestIndex and m_calcIndex once per update so the "newest"
        // and "middle" array slots move with time
        m_newestIndex = (m_newestIndex + 1) % 5;
        m_calcIndex = (m_calcIndex + 1) % 5;

        // --- Apply 2nd order Savitzky–Golay filtering (standard {mid reference} 5‑point window) 
        // These calculate and smooth the derivitives which yield linear and angular
        // velocity, acceleration, and jerk ---

        // Linear velocity magnitude 
        double dx = sgFirstDerivative(m_xHist, m_idx, dt);
        double dy = sgFirstDerivative(m_yHist, m_idx, dt);
        m_vel = Math.hypot(dx, dy);
 
        // Linear acceleration magnitude
        double ax = sgSecondDerivative(m_xHist, m_idx, dt); 
        double ay = sgSecondDerivative(m_yHist, m_idx, dt);
        m_accel = Math.hypot(ax, ay);
 
        // Linear jerk
        double jx = sgThirdDerivative(m_xHist, m_idx, dt); 
        double jy = sgThirdDerivative(m_yHist, m_idx, dt);
        m_jerk = Math.hypot(jx, jy);

        // Angular velocity and acceleration from Odometry
        m_angVelOdom = sgFirstDerivative(m_thetaHistOdom, m_idx, dt);
        m_angAccelOdom = sgSecondDerivative(m_thetaHistOdom, m_idx, dt);
        m_angJerkOdom = sgThirdDerivative(m_thetaHistOdom, m_idx, dt);
 
        // Angular velocity, acceleration and jerk from Gyro
        // (yes, gyro provides angVel directly, but we calculate it here anyway
        // from the Yaw samples to gain the same SG filtering as used for
        // Odometry samples).
        m_angVelGyro = sgFirstDerivative(m_thetaHistGyro, m_idx, dt);
        m_angAccelGyro = sgSecondDerivative(m_thetaHistGyro, m_idx, dt);
        m_angJerkGyro = sgThirdDerivative(m_thetaHistGyro, m_idx, dt);

        // Track maxima
        m_maxVel = Math.max(m_maxVel, Math.abs(m_vel));
        m_maxAccel = Math.max(m_maxAccel, Math.abs(m_accel));
        m_maxJerk = Math.max(m_maxJerk, Math.abs(m_jerk));
        m_maxAngVelOdom = Math.max(m_maxAngVelOdom, Math.abs(m_angVelOdom));
        m_maxAngAccelOdom = Math.max(m_maxAngAccelOdom, Math.abs(m_angAccelOdom));
        m_maxAngJerkOdom = Math.max(m_maxAngJerkOdom, Math.abs(m_angJerkOdom));
        m_maxAngVelGyro = Math.max(m_maxAngVelGyro, Math.abs(m_angVelGyro));
        m_maxAngAccelGyro = Math.max(m_maxAngAccelGyro, Math.abs(m_angAccelGyro));
        m_maxAngJerkGyro = Math.max(m_maxAngJerkGyro, Math.abs(m_angJerkGyro));

        // Put data to smart dashboard in case we want to graph the data 
        // Maximums are already published in the SwerveDrive Tab
        SmartDashboard.putNumber("Linear Vel = ", m_vel);
        SmartDashboard.putNumber("Linear Accel = ", m_accel);
        SmartDashboard.putNumber("Linear Jerk = ", m_jerk);
        SmartDashboard.putNumber("Ang Vel Odom = ", m_angVelOdom);
        SmartDashboard.putNumber("Ang Accel Odom = ", m_angAccelOdom);
        SmartDashboard.putNumber("Ang Jerk Odom = ", m_angJerkOdom);
        SmartDashboard.putNumber("Ang Vel Gyro = ", m_angVelGyro);
        SmartDashboard.putNumber("Ang Accel Gyro = ", m_angAccelGyro);
        SmartDashboard.putNumber("Ang Jerk Gyro = ", m_angJerkGyro);
    }

    // --- Savitzky–Golay helpers ---

    //
    // unwrapHeading() returns an angle in radians representing an adjusted current heading
    // data sample, avoiding a potential sharp transition from -PI to PI, and vice versa. 
    // Note that heading angle getters for FRC Odometry and Gryo have been written to 
    // always return values in the range -PI to PI, so we need to handle the condition where
    // successive samples lie on opposite sides of that transition, despite being just a few 
    // degrees (or fractions of a radian) apart.
    // The adjustmnent basically just allows the returned angle to exceed the -PI to PI range
    // while avoiding any sharp discontinuity for derivative calculation purposes.
    // Arguments:
    // last = the last (i.e. previous) unwrappedHeading value
    // current = the current heading, exactly as read from either Odometry or the Gyro.
    // Returns:
    // The current heading transformed into an unwrappedHeading. This will just be the original
    // sample if the -PI to PI The difference between the boundary has not been crossed.
    // Otherwise it will be the same angle but with a value that is less than -PI, or greater
    // that +PI so that the difference between the last unwrappedHeading sample and the returned 
    // current unwrappedHeading sample will never exceed PI (safe to assert since the sample period
    // of 20 ms and robot response time precludes valid heading changes even approaching that limit
    // sample to sample.
    //
    private double unwrapHeading(double lastUnwrapped, double lastRaw, double currentRaw) { 
        double delta = currentRaw - lastRaw; 
        // Map delta into (-pi, pi] 
        if (delta > Math.PI) {
            delta -= 2.0 * Math.PI;
        } else if (delta < -Math.PI) {
            delta += 2.0 * Math.PI;
        }
        return lastUnwrapped + delta;
    }

    /** 
     * Calculate first derivative using standard 5‑point 2nd order SG filter. 
     * A "causal" filter uses only current and past data. A standard filter
     * waits to do the calculation until future samples (in this case, 2 subsequent
     * samples) are known, and the coefficients are based on a window where the
     * calculated output applies to the center sample period, i.e. is in phase with 
     * the center data sample.
     * h[] is the array of samples, in a sorted order, but the oldest can be in any
     * location. The i[] array holds the key: i[0] is always the index of the oldest
     * sample in the history array, and i[4] is always the index of the newest sample in
     * the history array. dt is the averaged sample time period (will be ~20 ms).
    */
    private double sgFirstDerivative(double[] h, int[] i, double dt) {
        // Was (1st order causal): return (-2*h[4] - 1*h[3] + 1*h[1] + 2*h[0]) / (10 * dt);
        return (h[i[0]] - 8 * h[i[1]] + 8 * h[i[3]] - h[i[4]]) / (12 * dt);
    }

    /** 
     * Calculate second derivative using 2nd order standard 5‑point SG filter
     * h[] is the array of samples, i[0] to i[4] contains the indexes of the 
     * array of samples, oldest to newest, and dt
     * is the average sample time period (~20 ms, expected variations are small)
     * over the window of samples.
     */
    private double sgSecondDerivative(double[] h, int[] i, double dt) {
        return (-h[i[0]] + 16 * h[i[1]] - 30 * h[i[2]] + 16 * h[i[3]] - h[i[4]]) / (12 * dt * dt);
    }
        
    /**
     * Calculate third derivative (jerk) using centered 5‑point Savitzky–Golay filter.
     *
     * h[]          - array of 5 samples
     * int[]        - array holding indices of oldest [0] to newest [4] samples, 
     * dt           - sample period
     *
     * Uses standard centered SG coefficients for 5‑point, 2nd‑order polynomial:
     *   jerk = (-h[-2] + 2*h[-1] - 2*h[+1] + h[+2]) / (2 * dt^3)
     */
    private double sgThirdDerivative(double[] h, int[] i, double dt) {
        return (-h[i[0]] + 2 * h[i[1]] - 2 * h[i[3]] + h[i[4]]) / (2.0 * dt * dt * dt);
    }

    // --- Getters ---

    public double getVelocity() { return m_vel; }
    public double getAcceleration() { return m_accel; }
    public double getJerk() { return m_jerk; }
    public double getAngVelOdom() { return m_angVelOdom; }
    public double getAngAccelOdom() { return m_angAccelOdom; }
    public double getAngJerkOdom() { return m_angJerkOdom; }
    public double getAngVelGyro() { return m_angVelGyro; }
    public double getAngAccelGyro() { return m_angAccelGyro; }
    public double getAngJerkGyro() { return m_angJerkGyro; }

    public double getMaxVelocity() { return m_maxVel; }
    public double getMaxAcceleration() { return m_maxAccel; }
    public double getMaxJerk() { return m_maxJerk; }
    public double getMaxAngVelOdom() { return m_maxAngVelOdom; }
    public double getMaxAngAccelOdom() { return m_maxAngAccelOdom; }
    public double getMaxAngJerkOdom() { return m_maxAngJerkOdom; }
    public double getMaxAngVelGyro() { return m_maxAngVelGyro; }
    public double getMaxAngAccelGyro() { return m_maxAngAccelGyro; }
    public double getMaxAngJerkGyro() { return m_maxAngJerkGyro; }

    public void resetMax() {
        // Currently called from SwerveSubsystem whenever the field orientaiton is set
        m_maxVel = 0.0;
        m_maxAccel = 0.0;
        m_maxJerk = 0.0;
        m_maxAngVelOdom = 0.0; 
        m_maxAngAccelOdom = 0.0;
        m_maxAngJerkOdom = 0.0;
        m_maxAngVelGyro = 0.0;
        m_maxAngAccelGyro = 0.0;
        m_maxAngJerkGyro = 0.0;
    }
}