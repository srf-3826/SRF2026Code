package frc.lib.Sensors;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.servohub.config.ServoHubConfigAccessor;
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

public class GyroIO {
    private Pigeon2 m_gyro;
    private boolean m_isInverted;
    private double m_startupTime = Timer.getFPGATimestamp();
    private boolean m_isInitialized =  false;
    private double INITIALIZATION_QUIET_TIME = 0.5;        // seconds

    // StatusSignals
    private final StatusSignal<Angle> m_yawSignal;
    private final StatusSignal<Angle> m_pitchSignal;
    private final StatusSignal<Angle> m_rollSignal;
    private final StatusSignal<LinearAcceleration> m_accelXSignal;
    private final StatusSignal<LinearAcceleration> m_accelYSignal;
    private final StatusSignal<AngularVelocity> m_angVelZDeviceSignal;
    private final StatusSignal<AngularVelocity> m_angVelZWorldSignal;
    private final StatusSignal<Temperature> m_gyroTempSignal;

    public GyroIO(int canID, boolean isInverted, CANBus canbusToUse) {
        m_gyro = new Pigeon2(canID, canbusToUse);
        m_isInverted = isInverted;                  // Not currently implemented, becuase
                                                    // SRF has standardized on Pigeon2,
                                                    // so it can just be stored and 
                                                    // ignored for now. But will need to 
                                                    // be implemented if
                                                    // NavX is once again used.
        // Create all signals immediately (safe even before Config - these do not
        // involve any CANbus traffic)
        m_yawSignal  = m_gyro.getYaw();
        m_pitchSignal = m_gyro.getPitch();
        m_rollSignal  = m_gyro.getRoll();
        m_accelXSignal = m_gyro.getAccelerationX();
        m_accelYSignal = m_gyro.getAccelerationY();
        m_angVelZDeviceSignal = m_gyro.getAngularVelocityZDevice();
        m_angVelZWorldSignal  = m_gyro.getAngularVelocityZWorld();
        m_gyroTempSignal = m_gyro.getTemperature();

        m_isInitialized = false;
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
    }

    //
    // Call gyro.update() once per robot loop - insert into Robot.java
    //
    public void update() {
        double now = Timer.getFPGATimestamp();      // First, read the time

        // During startup period: never refresh
        if (now - m_startupTime < INITIALIZATION_QUIET_TIME) {
            // For the first INITIALIZATION_QUIET_TIME seconds, suppress error messages from
            // the Pigeon2 by ensuring no refresch calls are made during this time.
            // Apparently only the refresh() or refreshAll() methods trigger error messages.
            // This will help keep the startup console quieter, hopefully.
            return; 
        }

        // After the startup period has passed, just assume Pigeon2 hardware initialization is 
        // sucessful and complete - either way, go ahead and start / continue refreshing, and 
        // report all errors encountered, but if our internal m_isInitialized flag isstill false,
        // there is a little bit of cogfiguring still to do,foloowed by clearing the flag.

        // Now, since we're past the startup time, do a refreshAll()
        StatusCode code = BaseStatusSignal.refreshAll( m_yawSignal, m_pitchSignal, m_rollSignal,
                                                       m_accelXSignal, m_accelYSignal,
                                                       m_angVelZDeviceSignal, m_angVelZWorldSignal,
                                                       m_gyroTempSignal
                                                     );

        // And do normal runtime error reporting, if appropriate
        if ( !code.isOK() ) {
            System.out.println("Pigeon2 runtime error: " + code.getDescription());
        } else {                        
            // No error, erverthing is OK. If init is not finished, do it now and report time
            if (! m_isInitialized) {
                configPigeon2Gyro();
                zeroGyro();
                m_isInitialized = true;
                // To fine tune how long it takes the Pigeon2 to boot, set INITIALIZE_QUIET_TIME 
                // to a very low number, or even 0
                System.out.println("Pigeon2 fully initialized at time: " + now);
            } else {
                // We're basically done, but...
                // Optional - publish gyro data here. If so, use the time already sampled to slow the 
                // print rate to a max of 10 hz. However, usually pertinent Gyro data will be 
                // published by those subsystem(s) that need the gyro sensor data to function...
                // So publishing here might be redundant
            }
        }
    }   

    // ------------------ Getters ------------------
    // If called before initialization is complete, they generall 
    // return either 0 (probably what we want to initialize mechanisms
    // with)), or the last data value cached.

    public double getYawDegrees() {
        return MathUtil.inputModulus(m_yawSignal.getValueAsDouble(), -180.0, 180.0);
    }

    public double getPitchDegrees() {
        return m_pitchSignal.getValueAsDouble();
    }

    public double getRollDegrees() {
        return m_rollSignal.getValueAsDouble();
    }

    public double getAccelerationX() {
        return m_accelXSignal.getValueAsDouble();
    }

    public double getAccelerationY() {
        return m_accelYSignal.getValueAsDouble();
    }

    public double getAngularVelocityZ_Device() {
        return m_angVelZDeviceSignal.getValueAsDouble();
    }

    public double getAngularVelocityZ_World() {
        return m_angVelZWorldSignal.getValueAsDouble();
    }

    public double getTemperatureC() {
        return m_gyroTempSignal.getValueAsDouble();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(0.0);     //getYawDegrees());
        // Might be possible to do a direct Rotation2d fetch: return m_gyro.getRotation2d();
        // but it is not clear how efficient that would be wrt canbus
        // optimiztion and refreshing
    }

    public Rotation3d getRotation3d() {
        return new Rotation3d(
            Math.toRadians(0.0),         //getRollDegrees()),
            Math.toRadians(0.0),         //getPitchDegrees()),
            Math.toRadians(0.0)         //getYawDegrees())
        );
    }
}

