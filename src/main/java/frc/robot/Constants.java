package frc.robot;

import java.text.DecimalFormat;
import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.swerve.SDS_SwerveUnitParams;
import frc.lib.swerve.SwerveModuleConstants;

public final class Constants {
    /***************************************************
     * Universal Constants
     ***************************************************/
    // public static final String CAN_BUS_FOR_SWERVE = "CANivore0";
    public static final String CAN_BUS_FOR_SWERVE = "CANivore0";
    public static final String CAN_BUS_FOR_TURRET = "rio";
    public static final String CAN_BUS_FOR_EVERYTHING_ELSE = "CANivore1";

    public static final class F {
        // Formatters to control number of decimal places 
        // in the various published / recorded data
        public static DecimalFormat df0 = new DecimalFormat("#.");
        public static DecimalFormat df1 = new DecimalFormat("#.#");
        public static DecimalFormat df2 = new DecimalFormat("#.##");
        public static DecimalFormat df3 = new DecimalFormat("#.###");
        public static DecimalFormat df4 = new DecimalFormat("#.####");
        public static DecimalFormat df20 = new DecimalFormat("##");
        public static DecimalFormat df30 = new DecimalFormat("###");
        public static DecimalFormat df40 = new DecimalFormat("####");
        public static DecimalFormat df80 = new DecimalFormat("########");
    }

    /****************************************************
     * User Interface Constants
     ****************************************************/
    public static final class UIC {             // UIC = short for UserInterfaceConstants
        public static final double JOYSTICK_DEADBAND = 0.18;
    }

    /*****************************************************
     * Gyro Constants
     *****************************************************/
    public static final class GC {              // GC = short for GyroConstants
        public static final int PIGEON_2_CANID = 1;
        public static final boolean INVERT_GYRO = false; // In phoenix6, Pigeon2.getAngle() api
                                                        // returns heading, but it increases with
                                                        // CW rotation, the opposite of WPILib
                                                        // convention, which is CCW+ CW-,
                                                        // So if using getAngle(), set this to true.
                                                        // However, getRotation2d() instead can be used
                                                        // to return angles suited to WPILib
                                                        // conventions, with no gyro inversion.
        public static final int GYRO_LIST_COL = 2;
        public static final int FIRST_GYRO_LIST_ROW = 0;
        public static final int GYRO_LIST_HGT = 4;
 
        // if NavX, use NavX library to init (no need for an ID), but be sure to
        // set INVERT_GYRO = true; // because NavX measures CCW as negative.
    }

    /******************************************************
     * Swerve Drive Constants
     ******************************************************/
    public static final class SDC {             // SDC = short for SwerveDriveConstants

        public static final double BILLET_WHEEL_DIA_INCHES = 4.0;
        public static final double COLSON_WHEEL_DIA_INCHES = 3.96;
        public static final double PARADE_WHEEL_DIA_INCHES = 3.75;

         // REBUILT frame is 29" W x 24" L
        public static final double REBUILT_TRACK_WIDTH = Units.inchesToMeters(23.75); 
        public static final double REBUILT_WHEEL_BASE = Units.inchesToMeters(18.75); 
        public static final double HALF_RTW = REBUILT_TRACK_WIDTH / 2.0;
        public static final double HALF_RWB = REBUILT_WHEEL_BASE / 2.0;

        public static final Translation2d R_FL = new Translation2d(HALF_RWB, HALF_RTW);                                    
        public static final Translation2d R_FR = new Translation2d(HALF_RWB, -HALF_RTW);
        public static final Translation2d R_BL = new Translation2d(-HALF_RWB, HALF_RTW);
        public static final Translation2d R_BR = new Translation2d(-HALF_RWB, -HALF_RTW);

        // Offsets for the center of rotation, if needed, are the same as the
        // Translation2d coordinates of each corner. The center of the robot is the default,
        // defined here.
        // See SwerveSubsystem. Corner rotations are especially useful for blocking 
        // while playing defense, and sometimes even for evasion while on offense.
        public static final Translation2d   REL_POS2D_CEN = new Translation2d(0, 0);

        // Wheel angles for Rebuilt "park" - makes it difficult to move, slide, or be pushed
        public static final double R_PARK_ANGLE_LEFT_DEG = 45.0;
        public static final double R_PARK_ANGLE_RIGHT_DEG = -45.0;

       public static SDS_SwerveUnitParams SDSMK4i_REBUILT(double wheelDiaInches){
            double wheelDiaM = Units.inchesToMeters(wheelDiaInches);
            Translation2d relPos2D_FL = R_FL;
            Translation2d relPos2D_FR = R_FR;
            Translation2d relPos2D_BL = R_BL;
            Translation2d relPos2D_BR = R_BR;
            // Caution! The published CCwd values ((CC wheel direction, in degrees) 
            // per swerve module as seen on the Shuffleboard SwerveDrive Tab
            // are corrected to absolute angles within each CANCoder according to 
            // the specified magnetic offsets entered here. So to change these and use
            // new readings from the modules (first alligning all 4 wheels to straight
            // ahead manually) you must first set all offsets to 0 here, then compile
            // and deploy AND THEN POWER CYCLE before reading the CCDeg values.
            // Newly written values are not used until a reboot! Once calibrated wheel 
            // offset values have been obtained, the same process applies: enter them
            // here, then compile, deploy, and POWER CYCLE once again. After that, 
            // they should remain stable until a change is needed, e.g., for a 
            // module replacement. A simpler approach is to use TunerX,  which has
            // some secret sauce it uses to change the MagnetOffset without a 
            // power cycle being needed. However, the process is to click on a circle
            // icon with the wheels oriented straight ahead, and they only report the
            // offset in rotations. It will always be a negative nummber, because the
            // cancoder range is [0-1), and that must be negated because the Cancoder
            // algorithm is to always add the MagnetOffset to the internal raw reading.
            // Multiply by 360 to get degrees. 
            Rotation2d absOffsetFL = Rotation2d.fromDegrees(-0.86450195 * 360.0);
            Rotation2d absOffsetFR = Rotation2d.fromDegrees(-0.95922851 * 360.0);
            Rotation2d absOffsetBL = Rotation2d.fromDegrees(-0.80126953 * 360.0);
            Rotation2d absOffsetBR = Rotation2d.fromDegrees(-0.20825195 * 360.0);
            double parkAngleLeftDeg = R_PARK_ANGLE_LEFT_DEG;
            double parkAngleRightDeg = R_PARK_ANGLE_RIGHT_DEG;
            double steerGearRatio = ((150.0 / 7.0) / 1.0);
            // L2+ Drive Gear Ratio
            double driveL2GearRatio = (5.9 / 1.0);
            double steerKP = 75;             // TODO - this is likely overkill, but don't really see oscillations.
            double steerKI = 0.0;
            double steerKD = 0.0;
            double steerKF = 0.0;
            InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
            InvertedValue steerMotorInvert = InvertedValue.Clockwise_Positive;
            SensorDirectionValue canCoderDir = SensorDirectionValue.CounterClockwise_Positive;
            
            return new SDS_SwerveUnitParams(wheelDiaM,
                                            relPos2D_FL,
                                            relPos2D_FR,
                                            relPos2D_BL,
                                            relPos2D_BR,
                                            absOffsetFL,
                                            absOffsetFR,
                                            absOffsetBL,
                                            absOffsetBR,
                                            parkAngleLeftDeg,
                                            parkAngleRightDeg,
                                            steerGearRatio, 
                                            driveL2GearRatio, 
                                            steerKP, 
                                            steerKI, 
                                            steerKD, 
                                            steerKF, 
                                            driveMotorInvert, 
                                            steerMotorInvert, 
                                            canCoderDir);
        }
        
        // Set CHOOSEN_MODULE to just one choosen Robot year static method
        public static final SDS_SwerveUnitParams CHOOSEN_MODULE =  
                                                    // Note: REBUILT uses billet wheels, but with new 
                                                    // tread the dia is already not 4.00" but 3.98". 
                                                    // This will wear quickly to something closer to 
                                                    // the COLSON standard diameter (3.96") so that is
                                                    // used here.
                                                    SDSMK4i_REBUILT(COLSON_WHEEL_DIA_INCHES);

        // Now use CHOOSEN_MODULE to initialize generically named (and thus universally usable)
        // constants but which are still specific to the given module type. For 2026 other
        // robot/module choices were deleted, as we don't have multiple robots simultaneously
        // active anymore. 
        public static final double WHEEL_DIAMETER_M = CHOOSEN_MODULE.WHEEL_DIAMETER_M;
        public static final double WHEEL_CIRCUMFERENCE_M = CHOOSEN_MODULE.WHEEL_CIRCUMFERENCE_M;
        public static final Translation2d REL_POS2D_FL = CHOOSEN_MODULE.REL_POS2D_FL;
        public static final Translation2d REL_POS2D_FR = CHOOSEN_MODULE.REL_POS2D_FR;
        public static final Translation2d REL_POS2D_BL = CHOOSEN_MODULE.REL_POS2D_BL;
        public static final Translation2d REL_POS2D_BR = CHOOSEN_MODULE.REL_POS2D_BR;
        public static final Rotation2d ABS_OFFSET_FL = CHOOSEN_MODULE.ABS_OFFSET_FL;
        public static final Rotation2d ABS_OFFSET_FR = CHOOSEN_MODULE.ABS_OFFSET_FR;
        public static final Rotation2d ABS_OFFSET_BL = CHOOSEN_MODULE.ABS_OFFSET_BL;
        public static final Rotation2d ABS_OFFSET_BR = CHOOSEN_MODULE.ABS_OFFSET_BR;
        public static final double PARK_ANGLE_LEFT_DEG = CHOOSEN_MODULE.PARK_ANGLE_LEFT_DEG;
        public static final double PARK_ANGLE_RIGHT_DEG = CHOOSEN_MODULE.PARK_ANGLE_LEFT_DEG;
        public static final double DRIVE_GEAR_RATIO = CHOOSEN_MODULE.DRIVE_GEAR_RATIO;
        public static final double STEER_GEAR_RATIO = CHOOSEN_MODULE.STEER_GEAR_RATIO;
        public static final double STEER_KP = CHOOSEN_MODULE.STEER_KP;
        public static final double STEER_KI = CHOOSEN_MODULE.STEER_KI;
        public static final double STEER_KD = CHOOSEN_MODULE.STEER_KD;
        public static final double STEER_KF = CHOOSEN_MODULE.STEER_KF;
        public static final InvertedValue STEER_MOTOR_INVERT = CHOOSEN_MODULE.STEER_MOTOR_INVERT;
        public static final InvertedValue DRIVE_MOTOR_INVERT = CHOOSEN_MODULE.DRIVE_MOTOR_INVERT;
        public static final SensorDirectionValue CANCODER_DIR = CHOOSEN_MODULE.CANCODER_DIR;

        // Set Swerve Kinematics.  The order is always FL, FR, BL, and BR, 
        // referenced to an origin at the center of the robot
        // as evidenced by following the signs of the sequential 
        // Translation2d(x, y) coordinates below:
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = 
                                            new SwerveDriveKinematics(REL_POS2D_FL, 
                                                                      REL_POS2D_FR,
                                                                      REL_POS2D_BL,
                                                                      REL_POS2D_BR);

        // Swerve Drive Constants which are independent of given modules and chassis:
        // AbsoluteSensorDiscontinuityPointValue is apparently not applicable to 
        // CANCoders with Phoenix6, so the following init is not applicable
        // (a range of [0, 1) is fixed?
        // CANCODER_RANGE = AbsoluteSensorDiscontinuityPointValue.Unsigned_0To1;
        public static final int CANCODER_RANGE = 1; 

        // Unit conversion factors. With Phoenix6, the gear ratios are handled by the
        // motor controllers, so that motor.getPosition() values (assuming sensor source is
        // the embedded motor encoder) track the actual mechanism rotations. Thus the 
        // conversion factor for drive system rotations to meters of travel, and for drive
        // rotational velocity (RPS) to linear velopcity (MPS), is just 
        // the driven wheel circumference.
        public static final double TALONFX_ROT_TO_M_FACTOR = WHEEL_CIRCUMFERENCE_M;
        public static final double MPS_TO_TALONFX_RPS_FACTOR = 1.0 / WHEEL_CIRCUMFERENCE_M;
        public static final double ANGLE_TO_ROTATION_FACTOR = 1.0 / 360.0;

        // Current Limiting motor protection - same for both module types
        public static final double  DRIVE_SUPPLY_CURRENT_LIMIT          = 60.0;
        public static final boolean DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT   = true;
        public static final double  DRIVE_STATOR_CURRENT_LIMIT          = 100.0;
        public static final boolean DRIVE_ENABLE_STATOR_CURRENT_LIMIT   = true;

        public static final double STEER_SUPPLY_CURRENT_LIMIT = 40.0;
        public static final double STEER_SUPPLY_CURRENT_THRESHOLD = 0.0;
        public static final double STEER_SUPPLY_CURRENT_TIME_THRESHOLD = 0.1;
        public static final boolean STEER_ENABLE_SUPPLY_CURRENT_LIMIT = true;
        public static final double STEER_STATOR_CURRENT_LIMIT = 60.0;
        public static final boolean STEER_ENABLE_STATOR_CURRENT_LIMIT = true;
        
        // Voltage compensation
        public static final double STEER_MOTOR_VOLTAGE_COMPENSATION = 12.0;

        // These values are used by the drive motor to ramp in open loop.
        // Team 364 found a small open loop ramp (0.25) helps with tread wear, 
        // avoiding tipping, etc. In closed loop control, it would probably be 
        // better to employ profiled PID controllers.
        public static final double OPEN_LOOP_RAMP_PERIOD = 1.00;
        public static final double CLOSED_LOOP_RAMP_PERIOD = 0.0;

        // Drive Motor PID Values
        public static final double DRIVE_KP = 0.1;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        // Drive Motor Characterization Values
        // Divide SYSID values by 12 to convert from volts to percent output for CTRE
        public static final double DRIVE_KS = (0.32 / 12);
        public static final double DRIVE_KV = (1.51 / 12);
        public static final double DRIVE_KA = (0.27 / 12);
        public static final double DRIVE_KG = 0.27;
        
        // Steer motor characterization values.
        public static final double STEER_KS = (0.32 /12);
        public static final double STEER_KV = (1.51 /12);
        public static final double STEER_KA = (0.27 /12);
        public static final double STEER_KG = 0.27;

        // Swerve Profiling Values for Robot
        // Best if getten by characterizing the robot, but these values worked
        // tolerably well in 2023 as is. Used during Auto mode moves, which are
        // tracked by Odometry.
        public static final double MAX_ROBOT_SPEED_M_PER_SEC = 5.5; // 4.96 theoretically 
        public static final double MAX_ROBOT_ANG_VEL_RAD_PER_SEC = 11.0; // 11.96 theoretically 

        // Swerve output fixed limit values for teleop control (reduce if
        // speeds are too fast for the experience level of the drive team).
        // (In Auto, tuning should set speeds to reasonable values, no need
        // to reduce them - in fact, just the opposite, want fastest possible
        // movements in Auto mode, consistent with safety).
        public static final double OUTPUT_DRIVE_LIMIT_FACTOR = 1.0;
        public static final double OUTPUT_ROTATE_LIMIT_FACTOR = 1.0;

        // When monitored while set at -1 to 1, seemed like the Steering PID output 
        // did not generate percent outputs greater than about .4
        public static final double MIN_STEER_CLOSED_LOOP_OUTPUT = -0.6;
        public static final double MAX_STEER_CLOSED_LOOP_OUTPUT = 0.6;

        /* Default Motor Neutral Modes */
        public static final NeutralModeValue STEER_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast; // SparkBaseConfig.IdleMode.kCoast;
        public static final NeutralModeValue DRIVE_MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;

        // Swerve rotate speed and extent during a parade WAVE function
        public static final double WAVE_SWERVE_ROTATE_SPEED = 0.1;      // Percent simulated joystick input
        public static final double WAVE_ROTATION_EXTENT = 70;           // Degree units per side, relative to 0 heading
        public static final long   WAVE_ROTATION_PAUSE_IN_MS = 2500;    // Time in MS to stop rotating at limit
                                                                        // and wave faster, before reverting to
                                                                        // normal wave and rotating to other side
                                                                        // See WaveCmd.java
                                                                        
        // Finally, declare constants to define and allow addressing the (typically 4)
        // individual module components, including the Shuffleboard coordinates
        // embedded in the associated ShuffleboardLayout objects created for each
        // module under the shared "SwerveDrive" Shuffleboard Tab
        public static final int FIRST_SWERVE_MOD_LIST_COL = 4;
        public static final int FIRST_SWERVE_MOD_LIST_ROW = 0;
        public static final int SWERVE_MOD_LIST_HGT = 4;
        public static final ShuffleboardTab sbt = Shuffleboard.getTab("SwerveDrive");

        // Front Left Module - Module 0
        public static final class FL_Mod0 {
            public static final int driveMotorID = 1;
            public static final int steerMotorID = 2;
            public static final int canCoderID   = 1;
            public static final Rotation2d angleOffset = ABS_OFFSET_FL;
            public static final ShuffleboardLayout sBE_Layout0 = 
                                    sbt.getLayout("FL_Mod0", BuiltInLayouts.kGrid)
                                       .withPosition(FIRST_SWERVE_MOD_LIST_COL + 0, 
                                                     FIRST_SWERVE_MOD_LIST_ROW)
                                       .withSize(1, SWERVE_MOD_LIST_HGT)
                                       .withProperties(Map.of(  "Number of columns", 1, 
                                                                "Number of rows", 12, 
                                                                "Label position", "LEFT" ));
            public static final SwerveModuleConstants MODULE_CONSTANTS = 
                                        new SwerveModuleConstants(driveMotorID, 
                                                                  steerMotorID, 
                                                                  canCoderID, 
                                                                  angleOffset,
                                                                  sBE_Layout0);
        }

        // Front Right Module - Module 1
        public static final class FR_Mod1 {
            public static final int driveMotorID = 3;
            public static final int steerMotorID = 4;
            public static final int canCoderID   = 2;
            public static final Rotation2d angleOffset = ABS_OFFSET_FR;                             
            public static final ShuffleboardLayout sBE_Layout1 = 
                                    sbt.getLayout("FL_Mod1", BuiltInLayouts.kGrid)
                                       .withPosition(FIRST_SWERVE_MOD_LIST_COL + 1, 
                                                     FIRST_SWERVE_MOD_LIST_ROW)
                                       .withSize(1, SWERVE_MOD_LIST_HGT)
                                       .withProperties(Map.of(  "Number of columns", 1, 
                                                                "Number of rows", 12, 
                                                                "Label position", "LEFT" ));
            public static final SwerveModuleConstants MODULE_CONSTANTS = 
                                        new SwerveModuleConstants(driveMotorID, 
                                                                  steerMotorID, 
                                                                  canCoderID, 
                                                                  angleOffset,
                                                                  sBE_Layout1);
        }
        
        /* Back Left Module - Module 2 */
        public static final class BL_Mod2 {
            public static final int driveMotorID = 5;
            public static final int steerMotorID = 6;
            public static final int canCoderID   = 3;
            public static final Rotation2d angleOffset = ABS_OFFSET_BL;
    public static final ShuffleboardLayout sBE_Layout2 = 
                                    sbt.getLayout("FL_Mod2", BuiltInLayouts.kGrid)
                                       .withPosition(FIRST_SWERVE_MOD_LIST_COL + 2, 
                                                     FIRST_SWERVE_MOD_LIST_ROW)
                                       .withSize(1, SWERVE_MOD_LIST_HGT)
                                       .withProperties(Map.of(  "Number of columns", 1, 
                                                                "Number of rows", 12, 
                                                                "Label position", "LEFT" ));
            public static final SwerveModuleConstants MODULE_CONSTANTS = 
                                        new SwerveModuleConstants(driveMotorID, 
                                                                  steerMotorID, 
                                                                  canCoderID, 
                                                                  angleOffset,
                                                                  sBE_Layout2);
        }

        /* Back Right Module - Module 3 */
        public static final class BR_Mod3 {
            public static final int driveMotorID = 7;
            public static final int steerMotorID = 8;
            public static final int canCoderID   = 4;
            public static final Rotation2d angleOffset = ABS_OFFSET_BR;
            public static final ShuffleboardLayout sBE_Layout3 = 
                                    sbt.getLayout("FL_Mod3", BuiltInLayouts.kGrid)
                                       .withPosition(FIRST_SWERVE_MOD_LIST_COL + 3, 
                                                     FIRST_SWERVE_MOD_LIST_ROW)
                                       .withSize(1, SWERVE_MOD_LIST_HGT)
                                       .withProperties(Map.of(  "Number of columns", 1, 
                                                                "Number of rows", 12, 
                                                                "Label position", "LEFT" ));
            public static final SwerveModuleConstants MODULE_CONSTANTS = 
                                        new SwerveModuleConstants(driveMotorID, 
                                                                  steerMotorID, 
                                                                  canCoderID, 
                                                                  angleOffset,
                                                                  sBE_Layout3);
        }
    }

    /***********************************************************
     * Autonomous Constants
     ***********************************************************/
    public static final class AutoC {       // AutoC = short for AutoConstants
        public static final double AUTO_MAX_SPEED_M_PER_SEC = 4.5;
        public static final double AUTO_MAX_ACCEL_M_PER_SEC2 = 3.5;      // was 3.0
        public static final double AUTO_MAX_ANG_VEL_RAD_PER_SEC = 6.5*Math.PI;
        public static final double AUTO_MAX_ANG_ACCEL_RAD_PER_SEC2 = 3.5*Math.PI;

        public static final double KP_X_CONTROLLER = 20.0;
        public static final double KI_X_CONTROLLER = 0.5;
        public static final double KP_Y_CONTROLLER = 20.0;
        public static final double KI_Y_CONTROLLER = 0.5;
        public static final double KP_THETA_CONTROLLER = 12.0;
        public static final double KI_THETA_CONTROLLER = 0.5;
    
        /* Constraint for the motion profiled robot angle controller */
        public static final TrapezoidProfile.Constraints K_THETA_CONTROLLER_CONSTRAINTS =
                            new TrapezoidProfile.Constraints(AUTO_MAX_ANG_VEL_RAD_PER_SEC, 
                                                             AUTO_MAX_ANG_ACCEL_RAD_PER_SEC2);

        public static final double AUTO_SPEED_FACTOR_GENERIC = 1.0;
        public static final double AUTO_ACCEL_FACTOR_GENERIC = 1.0;
    }


    //Vision Constants
    public static final class VC {

        /**
        * Physical location of the camera on the robot, relative to the center of the robot.        
        */
        public static final Transform3d CAMERA_TO_ROBOT =
        new Transform3d(new Translation3d(12, 0.0, 0), new Rotation3d());
    
        public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();

        public static final double APRILTAG_AREA_IN = 42.25;

        public static final double MEASURED_APRILTAG_AREA = 25; // 1 foot away

        //public static final double 
        public static final double CAM_HEIGHT = 24;
        public static final double APRILTAG_HEIGHT = 24;
        public static final double PROCESSOR_HEIGHT = 47.875;

        public static final Double OFFSET_3_FT = 3.0;

    }
}
