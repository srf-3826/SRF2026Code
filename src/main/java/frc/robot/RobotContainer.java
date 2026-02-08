package frc.robot;

import com.ctre.phoenix6.CANBus;

// import java.util.function.Supplier;

// import org.photonvision.PhotonCamera;

// import edu.wpi.first.math.estimator.PoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RepeatCommand;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Sensors.GyroIO;
// import frc.robot.autos.*;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;
// import frc.robot.LimelightHelpers.LimelightResults;
// import frc.robot.subsystems.ClimbSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private CANBus swerveCanbus = new CANBus(Constants.CAN_BUS_FOR_SWERVE);
    private CANBus turretCanbus = new CANBus(Constants.CAN_BUS_FOR_TURRET);
    private CANBus allElseCanbus = new CANBus(Constants.CAN_BUS_FOR_EVERYTHING_ELSE);

    // Declare subsystem object handles
    private GyroIO                 m_gyroIO;
    private SwerveSubsystem        m_swerveSubsystem;
    // private VisionSubsystem     m_visionSubsystem;
    // private LimelightResults    limelight;
    // private Supplier<Pose2d>    m_robotPoseSupplier = ()-> m_swerveSubsystem.getPose();
    // private ClimbSubsystem      m_climbSubsystem;

    // Declare choosable autonomous Commands and any other Commands used with ButtonBindings
    private SwerveParkCmd       m_parkCmd;

    // Create sendable choosers for starting position and desired Auto routine
    // private static SendableChooser<Command> m_autoRoutineChooser = new SendableChooser<>();

    // Declare CommandXboxController
    private static CommandXboxController m_xbox;

    //  Constructor for the robot container. Contains subsystems, OI devices, and commands.
    public RobotContainer() {
        m_xbox = new CommandXboxController(0);
        m_gyroIO = new GyroIO(GC.PIGEON_2_CANID, GC.INVERT_GYRO, swerveCanbus);
        m_swerveSubsystem = new SwerveSubsystem(m_gyroIO, swerveCanbus);
        // m_poseEstimatorSubsystem = new PoseEstimatorSubsystem(limelight, m_swerveSubsystem);
        // m_visionSubsystem = new VisionSubsystem(limelight, m_swerveSubsystem);
        // m_climbSubsystem = new ClimbSubsystem();

        m_swerveSubsystem.setDefaultCommand(
                new DefaultDriveCmd(m_swerveSubsystem,
                                    // Xbox stick forward and stick to left are both neg, 
                                    // so must negate all for WPILib expected coordinates:
                                    () -> -m_xbox.getLeftY(),    // Y = translate: +fore / -back
                                    () -> -m_xbox.getLeftX(),    // X = strafe: +left / -right
                                    () -> -m_xbox.getRightX())); // rotate: +CCW / -CW

                // Park Cmd exits on any joystick input, so need to pass it all joystick input lambdas
                m_parkCmd = new SwerveParkCmd(m_swerveSubsystem,
                                      () -> -m_xbox.getLeftY(),
                                      () -> -m_xbox.getLeftX(),
                                      () -> -m_xbox.getRightX());
    /*
        m_autoRoutineChooser.setDefaultOption("Default Name", m_defaultAutoCmd);
        SmartDashboard.putData("Autonomous Selection:", m_autoRoutineChooser);
    */
        configureButtonBindings();
    }
    
    /**************************************************************
     * Getters for all subsystem Classes and other useful objects
     **************************************************************/
    public static XboxController getHidXboxCtrl() {
        return m_xbox.getHID();
    }

    public GyroIO getGyroIO() {
        return m_gyroIO;
    }

    /***********************************************
     * Button Bindings defines the operator UI
     ***********************************************/
    private void configureButtonBindings() {
        final Trigger ALT = m_xbox.leftBumper();

        // The following assignments govern the Xbox Controller UI for the Rebuilt Season:
        
        //    Left Bumbper (ALT)    => ALT mode: changes other buttons when pressed. No action on its own
        //    Right Bumper          => Slow mode (1/3 speed) while held
        //    ALT + Right bumper    => Super slow (1/10 speed) while held
        //    b                     => 
        //    ALT + b               => 
        //    y                     => 
        //    ALT + y               => 
        //    a                     => 
        //    ALT + a               => 
        //    x                     => Initiate Park (cross wheel angles), until any joystick input
        //    ALT + x               => Cancel any fuel action in progress. Retract intake if out.
        //    RightTrigger          => Rotate around right front swerve module while held
        //    ALT + RightTrigger    => Rotate around right rear swerve module while held
        //    LefTrigger            => Rotate around left front swerve module while held
        //    ALT + LeftTrigger     => Rotate around left rear swerve module while held
        //    L Joystick Y          => Swerve Translate (move fore/aft)
        //    L Joystick X          => Swerve Strafe (move side to side)
        //    R Joystick X          => Swerve Rotate (left = CCW, right = CW)
                                        // Rotation is about the center of the robot unless 
                                        // chnaged to rotation about a specific robot corner by 
                                        // holding left, right, ALT left, or ALT right
                                        // triggers. If both triggers held, left takes priority
        //    R Joystick Y          =>
        //    L Joystick Button     => Set Field Oriented drive. If already FO, ignore.
        //    R Joystick Button     => Set Robot Oriented drive. If already RO, ignore.
        //    Back                  => Zero the Gyro
        //    ALT + Back            => 
        //    Start                 => 
        //    ALT + Start           => 
        //    POV_UP                => 
        //    ALT + POV_UP          => 
        //    POV_DOWN              => 
        //    ALT + POV_DOWN        => 
        //    POV_LEFT              => 
        //    ALT + POV_LEFT        => 
        //    POV_RIGHT             => 
        //    ALT + POV_RIGHT       => 
        //    ALT+LJoyStickButton   => 
        //    ALT+RJoystickButton   => 
        
        // The following methods can be used to trigger rotation about any given corner. Most useful when
        // playing defense. Retained here in case a quick change to defense at competition
        // is needed, but normally these button bindings are needed for offense.
        m_xbox.leftTrigger().and(ALT.negate()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setFLCenOfRotation()));
        m_xbox.leftTrigger().and(ALT.negate()).onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        m_xbox.rightTrigger().and(ALT.negate()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setFRCenOfRotation()));                                        
        m_xbox.rightTrigger().and(ALT.negate()).onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));  
        ALT.and(m_xbox.leftTrigger()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setBLCenOfRotation()));
        ALT.and(m_xbox.leftTrigger()).onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        ALT.and(m_xbox.rightTrigger()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setBRCenOfRotation()));
        ALT.and(m_xbox.rightTrigger()).onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        
        // Left and right joystick buttons determine field oriented or robot oriented driving
        m_xbox.leftStick().and(ALT.negate()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setFieldOriented(true)));
        m_xbox.rightStick().and(ALT.negate()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setFieldOriented(false)));
        
        // Zero Gyro
        m_xbox.back().and(ALT.negate()).onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro()));   // was resetModulesToAbsolute()));

        // Right bumper alone = slow mode.
        // Alt + Right Bumper = super slow mode.
        // On Right bumper release (regardless of Left Bumper state), full speed.
        m_xbox.rightBumper().and(ALT.negate()).onTrue(
                new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(.5)));
        ALT.and(m_xbox.rightBumper()).onTrue(
                new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(.2)));
        m_xbox.rightBumper().onFalse(
                new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(1.0)));

//        m_xbox.x().and(ALT.negate()).onTrue(new InstantCommand(()-> (cancelAction));
        // Swerve park 
         ALT.and(m_xbox.x()).onTrue(m_parkCmd);

/*
        m_xbox.povLeft().and(ALT.negate()).onTrue(new InstantCommand(()-> );
        m_xbox.povRight().and(ALT.negate()).onTrue(new InstantCommand(()-> );
        m_xbox.povUp().and(ALT.negate()).onTrue(new InstantCommand(()-> );
        m_xbox.povDown().and(ALT.negate()).onTrue(new InstantCommand(()-> );

        // climb activities all require ALT button combination
        ALT.and(m_xbox.back()).onTrue(new InstantCommand(()->m_climbSubsystem.overrideEndOfMatchSafety()));
        ALT.and(m_xbox.povUp()).onTrue(new InstantCommand(()-> m_climbSubsystem.raise()));
        ALT.and(m_xbox.povUp()).onFalse(new InstantCommand(()-> m_climbSubsystem.stop()));
        ALT.and(m_xbox.povDown()).onTrue(new InstantCommand(()-> m_climbSubsystem.lower()));
        ALT.and(m_xbox.povDown()).onFalse(new InstantCommand(()-> m_climbSubsystem.stop()));
        ALT.and(m_xbox.leftTrigger()).onTrue(new InstantCommand(()-> m_climbSubsystem.runClimbWinch()));
        ALT.and(m_xbox.leftTrigger()).onFalse(new InstantCommand(()-> m_climbSubsystem.stopClimbWinch()));
*/
    }

    /*
     * getSelectedAutoCommand is called from Robot.AutonomousInit(),
     */
    public Command getSelectedAutoCommand() {
/*
        Command selectedAuto = m_autoRoutineChooser.getSelected();

        if (selectedAuto == null) {
            selectedAuto = new DoNothingCmd();
        }

        return selectedAuto;
*/
        return null;
    }
}