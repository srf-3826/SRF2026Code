package frc.lib.Sensors;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * WARNING! GyroIO is still under development. Not working right as
 * of Feb 4, 2026. As a separate class, for some reason, 
 * it does not play well on the CAN bus, resulting in stale frames.
 * Works in test programs, so not a hardware or wiring probllem.
 */
public class GyroIO {
    private Pigeon2 m_gyro;
    private boolean m_isInverted;
    double m_now = Timer.getFPGATimestamp();
    private boolean m_isWorking = false;

    // StatusSignals
    private final StatusSignal<Angle> m_yawSignal;
/*
    private final StatusSignal<Angle> m_pitchSignal;
    private final StatusSignal<Angle> m_rollSignal;
    private final StatusSignal<LinearAcceleration> m_accelXSignal;
    private final StatusSignal<LinearAcceleration> m_accelYSignal;
    private final StatusSignal<AngularVelocity> m_angVelZDeviceSignal;
    private final StatusSignal<AngularVelocity> m_angVelZWorldSignal;
    private final StatusSignal<Temperature> m_gyroTempSignal;
*/
    public GyroIO(int canID, boolean isInverted, CANBus canbusToUse) {
        m_gyro = new Pigeon2(canID, canbusToUse);
        m_isInverted = isInverted;                  // Not currently implemented, becuase
                                                    // SRF has standardized on Pigeon2,
                                                    // so it can just be stored and 
                                                    // ignored for now. But will need to 
                                                    // be implemented if
                                                    // NavX is once again used.
        // Create all signals immediately (safe even before Config)
        m_yawSignal  = m_gyro.getYaw();
/*
        m_pitchSignal = m_gyro.getPitch();
        m_rollSignal  = m_gyro.getRoll();
        m_accelXSignal = m_gyro.getAccelerationX();
        m_accelYSignal = m_gyro.getAccelerationY();
        m_angVelZDeviceSignal = m_gyro.getAngularVelocityZDevice();
        m_angVelZWorldSignal  = m_gyro.getAngularVelocityZWorld();
        m_gyroTempSignal = m_gyro.getTemperature();
*/
        // Wait for the Pigeon2 to come online (suppress startup errors)
        waitForStartup();
        configPigeon2Gyro();
    }

    //
    // Wait for the Gyro to finish booting, suppressing all transient error 
    // messages to the terminal, but do annouce when it fails to launch in a 
    // timely manner.
    //
    private void waitForStartup() {
        StatusCode code;

        while (!m_isWorking) {
            m_yawSignal.refresh();
            code = m_yawSignal.getStatus();
            m_now = Timer.getFPGATimestamp();
            SmartDashboard.putNumber("Yaw on startup", m_gyro.getYaw().getValueAsDouble());
            SmartDashboard.putNumber("Startup Time", m_now);
            if (code.isOK()) {
                m_isWorking = true;
            } else if (m_now < 30.0) {      // expexted boot time is a few seconds at most. Wait 30 sec during debug
                System.out.println("Pigeon2 still attempting to start @: " +
                                    Timer.      getFPGATimestamp()+"  "+code.getDescription()); 
                Timer.delay(0.05);// Wait 50 ms
            } else {
                System.out.println("Pigeon2 failed to start: ");
                break;
            }
        }
    }
    
    // The following configures the gyro, ensures it is zeroed, and optimizesBusUtilization.

    private void configPigeon2Gyro() {
      // Apply configuration (optional but recommended)
        Pigeon2Configuration p2Config = new Pigeon2Configuration();
        // Add any special Pigeon2 config here if needed
        StatusCode code = m_gyro.getConfigurator().apply(p2Config);
        if (! code.isOK()) {
            System.out.println("Pigeon2 config error: " + code.getDescription());
        }

        zeroGyro();
        m_gyro.optimizeBusUtilization();
    }

    //
    // Reset the gyro - current heading becomes new Zero
    //
    public void zeroGyro() {
        m_gyro.reset();
        m_yawSignal.refresh();
    }

    //
    // Call gyro.update() once per robot loop - insert into Robot.java
    //
    public void update() {
        StatusCode code;
        if (m_isWorking) {
           code = BaseStatusSignal.refreshAll( m_yawSignal 
                                                    //, m_pitchSignal, m_rollSignal,
                                                    // m_accelXSignal, m_accelYSignal,
                                                    // m_angVelZDeviceSignal, m_angVelZWorldSignal,
                                                    // m_gyroTempSignal
                                                );
            if (! code.isOK()) {
                System.out.println("Pigeon2 runtime error: " + code.getDescription());
            }
            // Optional - publish gyro data here. If so, put in a timer to slow the 
            // rate to a max of 10 hz. However, usually pertinent Gyro data will be 
            // published by those subsystem(s) that need gyro sensor data to function...
        }
    }

    // ------------------ Getters ------------------

    public double getYawDegrees() {
        return MathUtil.inputModulus(m_yawSignal.getValueAsDouble(), -180.0, 180.0);
    }
/*
    public double getPitchDegrees() {
        return 0.0;  //m_pitchSignal.getValueAsDouble();
    }

    public double getRollDegrees() {
        return 0.0;  //m_rollSignal.getValueAsDouble();
    }

    public double getAccelerationX() {
        return 0.0;  //m_accelXSignal.getValueAsDouble();
    }

    public double getAccelerationY() {
        return 0.0;  //m_accelYSignal.getValueAsDouble();
    }

    public double getAngularVelocityZ_Device() {
        return 0.0;  //m_angVelZDeviceSignal.getValueAsDouble();
    }

    public double getAngularVelocityZ_World() {
        return 0.0;  //m_angVelZWorldSignal.getValueAsDouble();
    }

    public double getTemperatureC() {
        return 0.0;  //m_gyroTempSignal.getValueAsDouble();
    }
*/
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(0.0);     //getYawDegrees());
        // Might be possible to do a direct: return m_gyro.getRotation2d();
        // but it is not clear how efficient that would be wrt canbus
        // optimiztion and refreshing
    }
/*
    public Rotation3d getRotation3d() {
        return new Rotation3d(
            Math.toRadians(0.0),         //getRollDegrees()),
            Math.toRadians(0.0),         //getPitchDegrees()),
            Math.toRadians(0.0)         //getYawDegrees())
        );
    }
*/
}

