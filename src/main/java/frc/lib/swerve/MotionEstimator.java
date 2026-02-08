// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

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
    private final double[] m_thetaHist = new double[5];

    private boolean m_initialized = false;
    private double  m_lastTime = Timer.getFPGATimestamp();

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
        double dt = now - m_lastTime;
        m_lastTime = now;

        if (!m_enabled || dt <= 0) {
            return;
        }

        double x = pose.getX();
        double y = pose.getY();
        double theta = pose.getRotation().getRadians();

        if (!m_initialized) {
            // Fill history with first sample
            for (int i = 0; i < 5; i++) {
                m_xHist[i] = x;
                m_yHist[i] = y;
                m_thetaHist[i] = theta;
            }
            m_initialized = true;
            return;
        }

        // Shift history (oldest drops off)
        shift(m_xHist, x);
        shift(m_yHist, y);
        shift(m_thetaHist, theta);

        // --- Savitzky–Golay filtering (causal 5‑point) ---

        // First derivative (velocity components)
        double dx = sgFirstDerivative(m_xHist, dt);
        double dy = sgFirstDerivative(m_yHist, dt);

        // Linear velocity magnitude
        m_vel = Math.hypot(dx, dy);

        // Linear acceleration magnitude
        double ax = sgSecondDerivative(m_xHist, dt); 
        double ay = sgSecondDerivative(m_yHist, dt);
        m_accel = Math.hypot(ax, ay);

        // Angular velocity and acceleration
        m_angVel = sgFirstDerivative(m_thetaHist, dt);
        m_angAccel = sgSecondDerivative(m_thetaHist, dt);

        // Track maxima
        m_maxVel = Math.max(m_maxVel, m_vel);
        m_maxVel = Math.max(m_maxAccel, Math.abs(m_accel));
        m_maxAngVel = Math.max(m_maxAngVel,Math.abs(m_angVel));
        m_maxAngAccel = Math.max(m_maxAngAccel, Math.abs(m_angAccel));
    }

    // --- Savitzky–Golay helpers ---

    private void shift(double[] hist, double newVal) {
        for (int i = 4; i > 0; i--) {
            hist[i] = hist[i - 1];
        }
        hist[0] = newVal;
    }

    /** First derivative using causal 5‑point SG filter */
    private double sgFirstDerivative(double[] h, double dt) {
        return (-3*h[4] - 2*h[3] - 1*h[2] + 1*h[1] + 2*h[0]) / (10 * dt);
    }

    /** Second derivative using causal 5‑point SG filter */
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
        m_maxVel = 
        m_maxAccel = 
        m_maxAngVel = 
        m_maxAngAccel = 0;
    }
}