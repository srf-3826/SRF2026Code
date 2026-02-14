package frc.lib.Sensors;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;

import java.util.Map;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.F;
import frc.robot.Constants.GC;

public class GyroIO {
    private Pigeon2     m_gyro;
    private boolean     m_isInverted;
    private double      m_now;
    private double      m_lastPubTime;
    private double      m_startupTime;
    private boolean     m_isInitialized =  false;
    private double      INITIALIZATION_QUIET_TIME = 0.3;        // seconds
    private double      PUBLISH_INTERVAL = 0.1;                 // Seconds 

    // StatusSignals
    private StatusSignal<Angle>                 m_yawSignal = null;
    private StatusSignal<Angle>                 m_pitchSignal = null;
    private StatusSignal<Angle>                 m_rollSignal = null;
    private StatusSignal<LinearAcceleration>    m_accelXSignal = null;
    private StatusSignal<LinearAcceleration>    m_accelYSignal = null;
    private StatusSignal<LinearAcceleration>    m_accelZSignal = null;
    private StatusSignal<Double>                m_gravityXSignal = null;
    private StatusSignal<Double>                m_gravityYSignal = null;
    private StatusSignal<Double>                m_gravityZSignal = null;
    private StatusSignal<AngularVelocity>       m_angVelZDeviceSignal = null;
    private StatusSignal<AngularVelocity>       m_angVelZWorldSignal = null;
    private StatusSignal<Temperature>           m_gyroTempSignal = null;

    // NT dashboard addresses for publishing
    private GenericEntry                        m_yawEntry;
    private GenericEntry                        m_pitchEntry;
    private GenericEntry                        m_rollEntry;
    private GenericEntry                        m_accelXEntry;
    private GenericEntry                        m_accelYEntry;
    private GenericEntry                        m_accelZEntry;
    private GenericEntry                        m_gravityXEntry;
    private GenericEntry                        m_gravityYEntry;
    private GenericEntry                        m_gravityZEntry;
    private GenericEntry                        m_angVelZDeviceEntry;
    private GenericEntry                        m_angVelZWorldEntry;
    private GenericEntry                        m_gyroTempEntry;

    public GyroIO(int canID, boolean isInverted, CANBus canbusToUse) {
        m_startupTime = Timer.getFPGATimestamp();
        m_gyro = new Pigeon2(canID, canbusToUse);
        m_isInverted = isInverted;                  // Not currently implemented - just stored.
                                                    // Needed only if NavX is once again used.
        m_isInitialized = false;
        setupGyroDataPublishing();
    }

    // The following configures the gyro, ensures it is zeroed, and optimizesBusUtilization.

    private boolean configPigeon2Gyro() {
      // Apply configuration (optional but recommended)
        Pigeon2Configuration p2Config = new Pigeon2Configuration();
        // Add any special Pigeon2 config here if needed
        StatusCode code = m_gyro.getConfigurator().apply(p2Config);
        if (! code.isOK()) {
            System.out.println("Pigeon2 config error: " + code.getDescription());
            return false;
        }

        zeroGyro();
        m_gyro.optimizeBusUtilization();
        return(true);
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
        m_now = Timer.getFPGATimestamp();      // First, read the time

        // During startup period: never refresh
        if (m_now - m_startupTime < INITIALIZATION_QUIET_TIME) {
            // For the first INITIALIZATION_QUIET_TIME seconds, suppress error messages from
            // the Pigeon2 by ensuring no gyro calls are made during this time.
            // This will help keep the startup console quieter, hopefully.
            return; 
        }

        // With quiet time compleete, check if init has been done yet. If not, do it now
        if (! m_isInitialized) {
            // Create all signals then config the gyro and clear the init flag
            m_yawSignal  = m_gyro.getYaw();
            m_pitchSignal = m_gyro.getPitch();
            m_rollSignal  = m_gyro.getRoll();
            m_accelXSignal = m_gyro.getAccelerationX();
            m_accelYSignal = m_gyro.getAccelerationY();
            m_accelZSignal = m_gyro.getAccelerationZ();
            m_angVelZDeviceSignal = m_gyro.getAngularVelocityZDevice();
            m_angVelZWorldSignal  = m_gyro.getAngularVelocityZWorld();
            m_gravityXSignal  = m_gyro.getGravityVectorX();
            m_gravityYSignal  = m_gyro.getGravityVectorY();
            m_gravityZSignal  = m_gyro.getGravityVectorZ();
            m_gyroTempSignal = m_gyro.getTemperature();
            if (m_yawSignal == null || m_pitchSignal == null || m_rollSignal == null ||
                m_accelXSignal == null || m_accelYSignal == null || m_accelZSignal == null ||
                m_angVelZDeviceSignal == null || m_angVelZWorldSignal == null ||
                m_gravityXSignal == null || m_gravityYSignal == null ||
                m_gravityZSignal == null || m_gyroTempSignal == null) {
                System.out.println("Failed to initialize Signals !!!");
            } else {
                if (configPigeon2Gyro()) {      // Returns true if statusCode == IS_OK
                   m_isInitialized = true;
                };
            }
            return;
        }
        
        // If this point is reached, Pigeon2 initialization has been done.
        // Go ahead and start / continue refreshing, and report any errors.
        StatusCode code = BaseStatusSignal.refreshAll( m_yawSignal, m_pitchSignal, m_rollSignal,
                                                       m_accelXSignal, m_accelYSignal, m_accelZSignal,
                                                       m_angVelZDeviceSignal, m_angVelZWorldSignal,
                                                       m_gravityXSignal, m_gravityYSignal, m_gravityZSignal,
                                                       m_gyroTempSignal);
        // And do normal runtime error reporting, if appropriate
        if ( !code.isOK() ) {
            System.out.println("GyroIO runtime error @ ~line 149. Time: "+m_now+"   "+code.getName());
        }
        publishGyroData();
    }   

    // ------------------ Getters ------------------
    // If called before initialization is complete, most getters rely on StatusSignals 
    // that are still null. Currently the only likely subsystem to start calling into 
    // gryoIO before init is complete is SwerveSubsystem, specifically getYawDegrees(). 
    // So test and just return 0 if that StatusSignal is null. (probably what we want 
    // to initialize Yaw with). NOTE: Will need to do the same with other signals if 
    // called by new mechanism subsystems early.

    public double getYawDegrees() {
        if (m_yawSignal != null) {
            return MathUtil.inputModulus(m_yawSignal.getValueAsDouble(), -180.0, 180.0);
        } else {
            return 0.0;
        }
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

    public double getAccelerationZ() {
        return m_accelZSignal.getValueAsDouble();
    }

    public double getAngularVelocityZ_Device() {
        return m_angVelZDeviceSignal.getValueAsDouble();
    }

    public double getAngularVelocityZ_World() {
        return m_angVelZWorldSignal.getValueAsDouble();
    }

    public double getGravityX() {
        return m_gravityXSignal.getValueAsDouble();
    }
    
    public double getGravityY() {
        return m_gravityYSignal.getValueAsDouble();
    }

    public double getGravityZ() {
        return m_gravityZSignal.getValueAsDouble();
    }


    public double getTemperatureC() {
        return m_gyroTempSignal.getValueAsDouble();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getYawDegrees());
        // Might be possible to do a direct Rotation2d fetch: return m_gyro.getRotation2d();
        // but it is not clear how efficient that would be wrt canbus
        // optimiztion and refreshing
    }

    public Rotation3d getRotation3d() {
        return new Rotation3d(Math.toRadians(getRollDegrees()),
                              Math.toRadians(getPitchDegrees()),
                              Math.toRadians(getYawDegrees()));
    }

    /*
     * Setup Gyro Data Publishing
     */
    // TODO: drop Gravity vector publishing. Replace with max horiz acceleration (calulated from X and Y accel),
    // max angular velocity, and max ang acceleration (calculated from derivative of angVel)
    private void setupGyroDataPublishing() {
        ShuffleboardTab sbt = Shuffleboard.getTab("SwerveDrive");
        if (sbt == null) {
            SmartDashboard.putString("GyroData Layout", "getLayout() Error occured");
        } else {
            ShuffleboardLayout gl2 = sbt.getLayout("Gyro Data", BuiltInLayouts.kGrid)
                                        .withPosition(GC.GYRO_LIST_COL, 
                                                      GC.FIRST_GYRO_LIST_ROW)
                                        .withSize(1, GC.GYRO_LIST_HGT)
                                        .withProperties(Map.of( "Number of columns", 1, 
                                                                "Number of rows", 12, 
                                                                "Label position", "LEFT" ));
            m_yawEntry              = gl2.add("Yaw", 0.0).withPosition(0, 0).getEntry();
            m_pitchEntry            = gl2.add("Pitch", 0.0).withPosition(0, 1).getEntry();
            m_rollEntry             = gl2.add("Roll", 0.0).withPosition(0, 2).getEntry();
            m_accelXEntry           = gl2.add("AccX", 0.0).withPosition(0, 3).getEntry();
            m_accelYEntry           = gl2.add("AccY", 0.0).withPosition(0, 4).getEntry();
            m_accelZEntry           = gl2.add("AccZ", 0.0).withPosition(0, 5).getEntry();
            m_angVelZDeviceEntry    = gl2.add("aVelD", 0.0).withPosition(0, 6).getEntry();
            m_angVelZWorldEntry     = gl2.add("aVelW", 0.0).withPosition(0, 7).getEntry();
            m_gravityXEntry         = gl2.add("GravX", 0.0).withPosition(0, 8).getEntry();
            m_gravityYEntry         = gl2.add("GravY", 0.0).withPosition(0, 9).getEntry();
            m_gravityZEntry         = gl2.add("GravZ", 0.0).withPosition(0, 10).getEntry();
            m_gyroTempEntry         = gl2.add("Temp", 0.0).withPosition(0, 11).getEntry();
        }
    }
    /*
     * Publish Gyro Data
     */
    private void publishGyroData() {
        if ((m_now - m_lastPubTime) < PUBLISH_INTERVAL) return;
        m_lastPubTime = m_now;

        m_yawEntry.setString(F.df1.format(getYawDegrees()));
        m_pitchEntry.setString(F.df1.format(getPitchDegrees()));
        m_rollEntry.setString(F.df1.format(getRollDegrees()));
        m_accelXEntry.setString(F.df2.format(getAccelerationX()));
        m_accelYEntry.setString(F.df2.format(getAccelerationY()));
        m_accelZEntry.setString(F.df2.format(getAccelerationZ()));
        m_angVelZDeviceEntry.setString(F.df2.format(getAngularVelocityZ_Device()));
        m_angVelZWorldEntry.setString(F.df2.format(getAngularVelocityZ_World()));
        m_gravityXEntry.setString(F.df2.format(getGravityX()));
        m_gravityYEntry.setString(F.df2.format(getGravityY()));
        m_gravityZEntry.setString(F.df2.format(getGravityZ()));
        m_gyroTempEntry.setString(F.df2.format(getTemperatureC()));
    }
}