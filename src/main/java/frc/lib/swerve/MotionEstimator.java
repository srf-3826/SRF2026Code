// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * MotionEstimator
 *
 * Computes linear velocity, linear acceleration, angular velocity,
 * and angular acceleration using odometry data and a causal
 * 5‑sample Savitzky–Golay filter.
 *
 * Designed for logging max performance during testing.
 */
public class MotionEstimator {

    // Enable/disable all computation (for competition mode)
    private boolean m_enabled = true;

    // History buffers (newest at index 0)
    private final double[] m_xHist = new double[5];
    private final double[] m_yHist = new double[5];
    private final double[] m_tHist = new double[5];
    private final double[] m_thetaHist = new double[5];

    private boolean m_initialized = false;
    private double  m_lastTime = Timer.getFPGATimestamp();
    private int     m_tIndex = 0;

    // Smoothed outputs
    private double m_vel = 0;
    private double m_accel = 0;
    private double m_angVel = 0;
    private double m_angAccel = 0;

    // Max tracking
    private double m_maxVel = 0;
    private double m_maxAccel = 0;
    private double m_maxAngVel = 0;
    private double m_maxAngAccel = 0;

    public MotionEstimator() {}

    /** Enable or disable computation */
    public void setEnabled(boolean enable) {
        this.m_enabled = enable;
    }

    /** Update estimator with the latest odometry pose */
    public void update(Pose2d pose) {
        double now = Timer.getFPGATimestamp();
        double dt = now - m_lastTime;           // dt on entry to update is set to the latest loop time - 
        m_lastTime = now;                       // later (after storing the latest loop time) dt will 
                                                // be set to the average of the last 5 loop times.
         if (!m_enabled || dt <= 0.0) {
            return;
        }

        double x = pose.getX();
        double y = pose.getY();
        double theta = pose.getRotation().getRadians();

        if (!m_initialized) {
            // Fill histories (all 5 entries) with initial samples: for x, y, and theta
            // Fill dt histopry with assumed fixed loop time of 20 ms (to avoid overly 
            // aged m_lastTime on startup).
            for (int i = 0; i < 5; i++) {
                m_xHist[i] = x;
                m_yHist[i] = y;
                m_thetaHist[i] = theta;
                m_tHist[i] = .02;
            }
            m_tIndex = 0;
            resetMax();
            m_initialized = true;
            return;
        }

        // Shift history (oldest drops off)
        shift(m_xHist, x);
        shift(m_yHist, y);
        shift(m_thetaHist, theta);
    
        // If m_tIndex is over 4, wrap to 0, then
        // Store the latest dt in the m_tIndex position, overwriting oldest sample,
        // then increment the index.
        // No need to preserve any other order, since all we need is the average 
        // on each new update calculation.
        if (m_tIndex > 4) m_tIndex = 0;
        m_tHist[m_tIndex++] = dt;
        
        // Now set dt to the AVERAGE of the last 5 dt samples
        dt = (m_tHist[0] + m_tHist[1] + m_tHist[2] + m_tHist[3] +  m_tHist[4]) /  5.0;

        // --- Apply 1st order Savitzky–Golay filtering (causal 5‑point) 
        // to calculate and smooth the derivitives which yield linear
        // velocity and acceleration ---

        // Linear velocity magnitude 
        double dx = sgFirstDerivative(m_xHist, dt);
        double dy = sgFirstDerivative(m_yHist, dt);
        m_vel = Math.hypot(dx, dy);
 
        // Linear acceleration magnitude
        double ax = sgSecondDerivative(m_xHist, dt); 
        double ay = sgSecondDerivative(m_yHist, dt);
        m_accel = Math.hypot(ax, ay);

        // Angular velocity and acceleration
        m_angVel = sgFirstDerivative(m_thetaHist, dt);
        m_angAccel = sgSecondDerivative(m_thetaHist, dt);

        // Track maxima
        m_maxVel = Math.max(m_maxVel, Math.abs(m_vel));
        m_maxAccel = Math.max(m_maxAccel, Math.abs(m_accel));
        m_maxAngVel = Math.max(m_maxAngVel,Math.abs(m_angVel));
        m_maxAngAccel = Math.max(m_maxAngAccel, Math.abs(m_angAccel));

        // Put data to smart dashboard in case we want to graph the data 
        // Maximums are already published in the SwerveDrive Tab
        SmartDashboard.putNumber("Calc Vel = ", m_vel);
        SmartDashboard.putNumber("Calc Accel = ", m_accel);
        SmartDashboard.putNumber("AngVel = ", m_angVel);
        SmartDashboard.putNumber("AngAccel = ", m_angAccel);
    }

    // --- Savitzky–Golay helpers ---

    private void shift(double[] hist, double newVal) {
        // Maintain sample order for distance (0 = latest data)
        hist[4] = hist[3];
        hist[3] = hist[2];
        hist[2] = hist[1];
        hist[1] = hist[0];
        hist[0] = newVal;
    }

    /** 
     * Calculate first derivative using causal 5‑point 1st order SG filter. A "causal" filter 
     * uses only current and past data. A 1st order filter just means linear.
     * h[] is the array of samples (index 0 to 4, newest to oldest) and dt
     * is the average sample time period (~20 ms). Expected variations should be small) 
    */
    private double sgFirstDerivative(double[] h, double dt) {
        // Was (from Copilot): return (-3*h[4] - 2*h[3] - 1*h[2] + 1*h[1] + 2*h[0]) / (10 * dt);
        // Corrected to:
        return (-2*h[4] - 1*h[3] + 1*h[1] + 2*h[0]) / (10 * dt);
    }

    /** 
     * Calculate second derivative using 2nd order causal 5‑point SG filter
     * h[] is the array of samples (index 0 to 4, newest to oldest) and dt
     * is the average sample time period (~20 ms, expected variations are small)
     */
    private double sgSecondDerivative(double[] h, double dt) {
        return (2*h[4] - 1*h[3] - 2*h[2] - 1*h[1] + 2*h[0]) / (7 * dt * dt);
    }

    // --- Getters ---

    public double getVelocity() { return m_vel; }
    public double getAcceleration() { return m_accel; }
    public double getAngularVelocity() { return m_angVel; }
    public double getAngularAcceleration() { return m_angAccel; }

    public double getMaxVelocity() { return m_maxVel; }
    public double getMaxAcceleration() { return m_maxAccel; }
    public double getMaxAngularVelocity() { return m_maxAngVel; }
    public double getMaxAngularAcceleration() { return m_maxAngAccel; }

    public void resetMax() {
        // Currently called from SwerveSubsystem whenever the field orientaiton is set
        m_maxVel = 
        m_maxAccel = 
        m_maxAngVel = 
        m_maxAngAccel = 0;
    }
}