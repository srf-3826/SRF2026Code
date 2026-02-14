package frc.robot.subsystems;

import frc.lib.swerve.MotionEstimator;
import frc.lib.swerve.SwerveModule;
import frc.lib.Sensors.GyroIO;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Map;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    private double              m_now;

    private Pose2d              m_location2d;
    private SwerveDriveOdometry m_swerveOdometry;
    private MotionEstimator     m_motionEstimator;
    private SwerveModule[]      m_swerveMods;
    private SwerveModuleState[] m_states = new SwerveModuleState[4];

    private GyroIO              m_gyro;
    private Rotation2d          m_currentHeading2d;

    private Translation2d       m_cenOfRotationOffset = SDC.REL_POS2D_CEN;
    private String              m_cenRotIdString;
    private boolean             m_isFieldOriented = true;       // default is Field Oriented on start

    // The following is a temporary driver settable speed reduction factor, normally
    // triggered (by Right Bumper, when held). When triggered, speed will be slower, for both 
    // translate and strafing, as well as rotation. Starts out as full speed.
    private static double       m_varMaxOutputFactor = 1.0;

    // The following are fixed (i.e. changable via re-compile only) reductions in the max speeds
    // allowed, to both increase driver control and reduce chance of damage, independent of
    // m_varMaxOutputFactor. The final throttle limits are the product of each with m_varOutputLimit.
    public double m_fixedMaxTranslationOutput  = SDC.OUTPUT_DRIVE_LIMIT_FACTOR;                  
    public double m_fixedMaxRotationOutput     = SDC.OUTPUT_ROTATE_LIMIT_FACTOR;                 
     
    private GenericEntry        m_isFieldOrientedEntry;
    public  GenericEntry        m_odometryPoseXEntry;
    public  GenericEntry        m_odometryPoseYEntry;
    private GenericEntry        m_odometryHeadingEntry;
    private GenericEntry        m_cenOfRotEntry;
    private GenericEntry        m_maxOutputFactorEntry;
    private GenericEntry        m_odometrySpeedEntry;
    private GenericEntry        m_odometryAngVelEntry;
    private GenericEntry        m_maxSpeedEntry;
    private GenericEntry        m_maxAngVelEntry;
    private GenericEntry        m_maxAccelEntry;
    private GenericEntry        m_maxAngAccelEntry;

    // Magic numbers ahead (publishing interval of 10 hz, and out of phasefactor). May want to move to
    // constants.java. Swerve modules will publish at the same rate as SwerveSubsystem, but out of 
    // phase by half the interval. 
    private static final double PUBLISH_INTERVAL = 0.01;            // Was .1, changed to Temporarily publish every loop
    private double              m_lastSwerveSubsystemPubTime = 0.0;
    private double              m_lastSwerveModulesPubTime = PUBLISH_INTERVAL / 2.0;
      
    public SwerveSubsystem(GyroIO gyro, CANBus swerveCanbus) {
        m_gyro = gyro;
        m_currentHeading2d = getYaw2d();

        m_swerveMods = new SwerveModule[] {
            new SwerveModule(0, SDC.FL_Mod0.MODULE_CONSTANTS, swerveCanbus),
            new SwerveModule(1, SDC.FR_Mod1.MODULE_CONSTANTS, swerveCanbus),
            new SwerveModule(2, SDC.BL_Mod2.MODULE_CONSTANTS, swerveCanbus),
            new SwerveModule(3, SDC.BR_Mod3.MODULE_CONSTANTS, swerveCanbus)
        };

        // By pausing init for a second before setting module offsets, we avoid 
        // a bug with inverting motors. This call is thread blocking, but is only 
        // called once at startup, so ignore.
        // See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
        Timer.delay(1.0);
        // Update 1/2024: also added waitForCANcoder method in SwerveModule, 
        // which does not hurt but according to above thread the delay is still 
        // needed, so restored that.
        resetModulesToAbsolute();

        resetCenOfRotation();

        m_swerveOdometry = new SwerveDriveOdometry(SDC.SWERVE_KINEMATICS, 
                                                   m_currentHeading2d,
                                                   getModulePositions());
        m_motionEstimator = new MotionEstimator();
        setupPublishing();
    }

    // The following five methods establish the center of rotation, initially or
    // on the fly, to either the center of the robot (default) or to one of the 
    // four module wheels.
    public void setFLCenOfRotation() {
        m_cenOfRotationOffset = SDC.REL_POS2D_FL;
        m_cenRotIdString = "FL";
    }

    public void setFRCenOfRotation() {
        m_cenOfRotationOffset = SDC.REL_POS2D_FR;
        m_cenRotIdString = "FR";
    }

    public void setBLCenOfRotation() {
        m_cenOfRotationOffset = SDC.REL_POS2D_BL;
        m_cenRotIdString = "BL";
    }

    public void setBRCenOfRotation() {
        m_cenOfRotationOffset = SDC.REL_POS2D_BR;
        m_cenRotIdString = "BR";
    }

    public void resetCenOfRotation() {
        m_cenOfRotationOffset = SDC.REL_POS2D_CEN;
        m_cenRotIdString = "Cen";
    }

    // drive() is the handler for teleop joystick driving, typically called from 
    // DefaultDriveCmd with isOpenLoop set to false.
    // It can also be called from PID controllers or other Commands as needed, 
    // typically with isOpenLoop set to false.
    public void drive(Translation2d translation, 
                      double rotation, 
                      boolean isOpenLoop) {
        translation = translation.times(m_varMaxOutputFactor * m_fixedMaxTranslationOutput);
        rotation = rotation * m_varMaxOutputFactor * m_fixedMaxRotationOutput;

        ChassisSpeeds chassisSpeeds = m_isFieldOriented 
                                        ?
                                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                            translation.getX(), 
                                            translation.getY(), 
                                            rotation, 
                                            m_currentHeading2d)
                                        :
                                        new ChassisSpeeds(
                                            translation.getX(), 
                                            translation.getY(), 
                                            rotation);
        SwerveModuleState[] swerveModuleStates =
            SDC.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds,
                                                       m_cenOfRotationOffset);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 
                                                    SDC.MAX_ROBOT_SPEED_M_PER_SEC);
        for (SwerveModule mod : m_swerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.m_modNum], isOpenLoop);
        }
    }  

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SDC.MAX_ROBOT_SPEED_M_PER_SEC);
        
        for(SwerveModule mod : m_swerveMods){
            mod.setDesiredState(desiredStates[mod.m_modNum], false);
        }
    }    

    public boolean isFieldOriented() {
        return m_isFieldOriented;
    }

    public void setFieldOriented( boolean fieldOrientedSetting ) {
        m_isFieldOriented = fieldOrientedSetting;
        // to reset the max vel and accel data, just change to either robot or field oriented. 
        // Then switch back to leave current field oriented setting unchanged 
        m_motionEstimator.resetMax();
    }

    public void setVarMaxOutputFactor(double maxOutputFactor) {
        if (maxOutputFactor < .1) {
            maxOutputFactor = .1;
        }
        if (maxOutputFactor > 1.0) {
            maxOutputFactor = 1.0;
        };
        m_varMaxOutputFactor = maxOutputFactor;
    }

    public static double getVarMaxOutputFactor() {
        return m_varMaxOutputFactor;
    } 

    public Pose2d getPose() {
        return m_swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose2d) {
        m_swerveOdometry.resetPosition(m_currentHeading2d, getModulePositions(), pose2d);
    }

    public double getRobotTranslateVel() {
        m_states = getModuleStates();
        return (SDC.SWERVE_KINEMATICS
                .toChassisSpeeds(m_states).vxMetersPerSecond);
    }
    
    public double getRobotStrafeVel() {
        m_states = getModuleStates();
        return (SDC.SWERVE_KINEMATICS
                .toChassisSpeeds(m_states).vyMetersPerSecond);
    }
    
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : m_swerveMods){
            states[mod.m_modNum] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : m_swerveMods){
            positions[mod.m_modNum] = mod.getModulePosition();
        }
        return positions;
    }

    public void zeroGyro() {
        m_gyro.zeroGyro();
        m_currentHeading2d = getYaw2d();
    }

    public Rotation2d getYaw2d() {
        return m_gyro.getRotation2d();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : m_swerveMods){
            mod.resetToAbsolute();
        }
    }

    public void setupPublishing() {
        ShuffleboardTab sbt = Shuffleboard.getTab("SwerveDrive");
        if (sbt == null) {
            SmartDashboard.putString("SwerveDrive Tab", "getTab() Error occured");
        } else {
            // Initialize column for Robot Data
            ShuffleboardLayout sl =  sbt.getLayout("RobotData", BuiltInLayouts.kGrid)
                                        .withPosition(0, 0)
                                        .withSize(1, SDC.SWERVE_MOD_LIST_HGT)
                                        .withProperties(Map.of("Number of Columns", 1,
                                                               "Number of Rows", 12, 
                                                               "Label position", "LEFT"));
            if (sl == null) {
                SmartDashboard.putString("RobotData Layout", "getLayout() Error occured");
            } else {
                m_isFieldOrientedEntry  = sl.add("Field Or", "Yes")
                                            .withPosition(0, 0)
                                            .getEntry();
                m_odometryPoseXEntry    = sl.add("Xpos m", F.df2.format(0.0))
                                            .withPosition(0, 1)
                                            .getEntry();
                m_odometryPoseYEntry    = sl.add("Ypos m", F.df2.format(0.0))
                                            .withPosition(0, 2)
                                            .getEntry();
                m_odometryHeadingEntry  = sl.add("Hdg D", F.df1.format(0.0))
                                            .withPosition(0, 3)
                                            .getEntry();
                m_cenOfRotEntry         = sl.add("C rot", m_cenRotIdString)
                                            .withPosition(0, 4)
                                            .getEntry();
                m_maxOutputFactorEntry  = sl.add("M out", F.df2.format(0.0))
                                            .withPosition(0, 5)
                                            .getEntry();
                m_odometrySpeedEntry    = sl.add("Spd", F.df2.format(0.0))
                                            .withPosition(0, 6)
                                            .getEntry();
                m_odometryAngVelEntry   = sl.add("@ Vel", F.df2.format(0.0))
                                            .withPosition(0, 7)
                                            .getEntry();
                m_maxSpeedEntry         = sl.add("M spd", F.df2.format(0.0))
                                            .withPosition(0, 8)
                                            .getEntry();
                m_maxAngVelEntry        = sl.add("M @vel", F.df2.format(0.0))
                                            .withPosition(0, 9)
                                            .getEntry();
                m_maxAccelEntry         = sl.add("Maccel", F.df2.format(0.0))
                                            .withPosition(0, 10)
                                            .getEntry();
                m_maxAngAccelEntry      = sl.add("M@accel", F.df2.format(0.0))
                                            .withPosition(0, 11)
                                            .getEntry();

                if ((m_isFieldOrientedEntry == null)
                    ||(m_odometryPoseXEntry == null)
                    ||(m_odometryPoseYEntry == null)
                    ||(m_odometryHeadingEntry == null)
                    ||(m_cenOfRotEntry == null)
                    ||(m_maxOutputFactorEntry == null)
                    ||(m_odometrySpeedEntry == null)
                    ||(m_odometryAngVelEntry == null)
                    ||(m_maxSpeedEntry == null)
                    ||(m_maxAngVelEntry == null)
                    ||(m_maxAccelEntry == null)
                    ||(m_maxAngAccelEntry == null)) {
                    SmartDashboard.putString("RobotData List Entries", "Null Entry handles(s) encountered");
                }
                // Initalize a column holding units for the adjacent Swerve Module Data Columns
                ShuffleboardLayout s2 =  sbt.getLayout("Units", BuiltInLayouts.kGrid)
                                            .withPosition(SDC.FIRST_SWERVE_MOD_LIST_COL + 4, 0)
                                            .withSize(2, SDC.SWERVE_MOD_LIST_HGT)
                                            .withProperties(Map.of("Number of Columns", 2,
                                                                    "Number of Rows", 12, 
                                                                    "Label position", "RIGHT"));
                // No need to cache the entries here - they only get written once in this setup method
                // Titles must be unique within this grid widget. Use variable len strings composed of
                // zero width non visible spaces. Default values are strings with the correct digits for 
                // adjacent Swerve Module data sets.
                s2.add(" "+makeInvisibleTitleOfLen(0), "DM  SM  CC")
                  .withPosition(0, 0);
                s2.add(" "+makeInvisibleTitleOfLen(1), "Deg - offsets")
                  .withPosition(0, 1);
                s2.add(" "+makeInvisibleTitleOfLen(2), "Deg - CC enc")
                  .withPosition(0, 2);
                s2.add(" "+makeInvisibleTitleOfLen(3), "Deg - SM enc")
                  .withPosition(0, 3);
                s2.add(" "+makeInvisibleTitleOfLen(4), "Deg - Setpoint")
                  .withPosition(0, 4);
                s2.add(" "+makeInvisibleTitleOfLen(5), "SM - PID out")
                  .withPosition(0, 5);
                s2.add(" "+makeInvisibleTitleOfLen(6), "Amps")
                  .withPosition(0, 6);
                s2.add(" "+makeInvisibleTitleOfLen(7), "Celcius")
                  .withPosition(0, 7);
                s2.add(" "+makeInvisibleTitleOfLen(8), "m/sec")
                  .withPosition(0, 8);
                s2.add(" "+makeInvisibleTitleOfLen(9), "meters")
                  .withPosition(0, 9);
                s2.add(" "+makeInvisibleTitleOfLen(10), "Amps")
                  .withPosition(0, 10);
                s2.add(" "+makeInvisibleTitleOfLen(11), "Celcius")
                  .withPosition(0, 11);
            }
        }
    }

    private static String makeInvisibleTitleOfLen(int index) {
        // "\u2008" is a non visible zero width Unicode space.
        return "\u200B".repeat(index + 1);
    }

    /*
     * This is where all SwerveSubsystem related data publishing gets scheduled and performed
     */
    public void publishSwerveSubsystemData() {
        // m_now is always set by periodic(), first thing
        if (m_now - m_lastSwerveSubsystemPubTime < PUBLISH_INTERVAL) return;  // 10 Hz
    
        // If this code is reached, it is itme to publish
        m_lastSwerveSubsystemPubTime = m_now;

        // class variable m_currentHeading2d is refreshed in periodic().
        // doing this should avoid making CAN bus data requests too frequently for
        // the Pigeon2 to keep up.
        m_isFieldOrientedEntry.setString(m_isFieldOriented ? "Yes" : "No");
        m_location2d = getPose();
        m_odometryPoseXEntry.setString(F.df2.format(m_location2d.getX()));
        m_odometryPoseYEntry.setString(F.df2.format(m_location2d.getY()));           
        m_odometryHeadingEntry.setString(F.df1.format(m_location2d.getRotation().getDegrees()));
        m_cenOfRotEntry.setString(m_cenRotIdString);
        m_maxOutputFactorEntry.setString(F.df2.format(m_varMaxOutputFactor * m_fixedMaxTranslationOutput));
        m_odometrySpeedEntry.setString(F.df2.format(m_motionEstimator.getVelocity()));
        m_odometryAngVelEntry.setString(F.df2.format(m_motionEstimator.getAngularVelocity())); 
        m_maxSpeedEntry.setString(F.df2.format(m_motionEstimator.getMaxVelocity())); 
        m_maxAngVelEntry.setString(F.df2.format(m_motionEstimator.getMaxAngularVelocity())); 
        m_maxAccelEntry.setString(F.df2.format(m_motionEstimator.getMaxAcceleration())); 
        m_maxAngAccelEntry.setString(F.df2.format(m_motionEstimator.getMaxAngularAcceleration())); 
    }

    /*
     * This is where all SwerveModule related data publishing gets scheduled and performed
     */
    public void publishSwerveModulesData() {
        // m_now is always set by periodic(), first thing
        if (m_now - m_lastSwerveModulesPubTime < PUBLISH_INTERVAL) return;
        for(SwerveModule mod : m_swerveMods) {
            mod.publishModuleData();
        }
        m_lastSwerveModulesPubTime = m_now;
    } 
    /*
        periodic is called on every loop instance. 
    */
    @Override
    public void periodic() {
        m_now = Timer.getFPGATimestamp();
        m_currentHeading2d = getYaw2d();            // cache the current gyro heading

        if (RobotState.isEnabled()) {
            m_swerveOdometry.update(m_currentHeading2d, getModulePositions()); 
            m_motionEstimator.update(m_swerveOdometry.getPoseMeters());
        }
    
        // Allow SwerveModules to all refresh their StatusSignals
        for(SwerveModule mod : m_swerveMods) {
            mod.update();
        }    
        publishSwerveSubsystemData();       // these eash have independent 
        publishSwerveModulesData();         // decimators to reduce publish frequency
    }

    // This is a test routine, designed to rotate all modules
    // synchronously to an identical specified heading in degrees
    public void rotateModulesToAngle(double angleDeg) {
        for(SwerveModule mod : m_swerveMods) {
            mod.setAngle(angleDeg);
        }    
    }

    // This is a method to cause all modules to be rotated to 
    // angles contained in an array passed as an argument, the
    // specified angles being in FL, FR, BL, and BR order.
    // Primarily used to set all modules to their PARK positions, 
    // but could be useful for other purposes.
    public void rotateModulesToAngles( double angleDeg[] ) {
        for(SwerveModule mod : m_swerveMods) {
            mod.setAngle(angleDeg[mod.m_modNum] );
        }
    }

    public void stop() {
        for(SwerveModule mod : m_swerveMods) {
            mod.stop();
        }            
    }

    public double getCurrentPose() {
        return m_swerveOdometry.getPoseMeters().getY();
    }
}