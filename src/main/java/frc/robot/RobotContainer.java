package frc.robot;

import com.ctre.phoenix6.CANBus;

// import choreo.auto.AutoRoutine;
//  edu.wpi.first.math.trajectory.Trajectory;

// import java.util.function.Supplier;

// import org.photonvision.PhotonCamera;

// import edu.wpi.first.math.estimator.PoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RepeatCommand;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.sensors.GyroIO;
// import frc.robot.autos.*;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
    private CANBus allElseCanbus = new CANBus(Constants.CAN_BUS_FOR_EVERYTHING_ELSE);

    // Declare subsystem object handles
    private GyroIO                 m_gyroIO;
    private SwerveSubsystem        m_swerveSubsystem;
    private IntakeSubsystem        m_intakeSubsystem;
    private ShooterSubsystem       m_shooterSubsystem;
    //private ClimbSubsystem         m_climbSubsystem;
    //private VisionSubsystem        m_visionSubsystem;
    //private PoseEstimatorSubsystem m_poseEstimatorSubsystem;

    // Declare commands needed for Teleop and Auto
    //private DeployIntakeCmd             m_DeployIntakeCmd;;
    //private RetractIntakeCmd            m_RetractIntakeCmd;
    //private RetractIntakeToMaximumCmd   m_RetractIntakeToMaximumCmd;
    //private EjectIntakeCmd              m_EjectIntakeCmd;
    private SwerveParkCmd               m_parkCmd;

    private DoNothingCmd                m_DoNothingCmd;
    //private MidfieldFullDefenseAuto     m_MidfieldFullDefenseAuto;
    //private LeftMidfieldDefenseAuto     m_LeftMidfieldDefenseAuto; 
    //private RightMidfieldDefenseAuto    m_RightMidfieldDefenseAuto; 
    //private ShootPreload                m_ShootPreload;
    //private ShootPreloadThenLMD         m_ShootPreloadThenLMD;
    //private ShootPreloadThenRMD         m_ShootPreloadThenRMD;
 
    // Decleare vision and odometry / trajectory following components
    //private LimelightResults            limelight;
    //private Supplier<Pose2d>            m_robotPoseSupplier = ()-> m_swerveSubsystem.getPose();
    //private Trajectory                  m_Trajectory;

    // Create sendable chooser for desired Auto routine
    private static SendableChooser<Command> m_autoRoutineChooser = new SendableChooser<>();

    // Declare CommandXboxController
    private static CommandXboxController m_xbox;

    //  Constructor for the robot container. Contains subsystems, OI devices, and commands.
    public RobotContainer() {
        m_xbox =             new CommandXboxController(0);
        m_gyroIO =           new GyroIO(GC.PIGEON_2_CANID, GC.INVERT_GYRO, swerveCanbus);
        m_swerveSubsystem =  new SwerveSubsystem(m_gyroIO, swerveCanbus);
        m_intakeSubsystem =  new IntakeSubsystem(allElseCanbus);
        m_shooterSubsystem = new ShooterSubsystem(allElseCanbus, m_intakeSubsystem);
        //m_poseEstimatorSubsystem = new PoseEstimatorSubsystem(limelight, m_swerveSubsystem);
        //m_visionSubsystem = new VisionSubsystem(limelight, m_swerveSubsystem);
        //m_climbSubsystem = new ClimbSubsystem();

        // Create commands used for teleop
        //m_RetractIntakeCmd = new RetractIntakeCmd(m_intakeSubsystem);
        //m_DeployIntakeCmd = new DeployIntakeCmd(m_intakeSubsystem);
        //m_RetractIntakeToMaximumCmd = new RetractIntakeToMaximumCmd(m_intakeSubsystem);
        //m_EjectIntakeCmd = new EjectIntakeCmd(m_intakeSubsystem);
        // Park Cmd exits on any joystick input, so need to pass it all joystick input lambdas
        m_parkCmd = new SwerveParkCmd(m_swerveSubsystem,
                                () -> -m_xbox.getLeftY(),
                                () -> -m_xbox.getLeftX(),
                                () -> -m_xbox.getRightX());

        // Create commands used for Auto
        m_DoNothingCmd =     new DoNothingCmd();
        //m_MidfieldFullDefenseAuto = new MidfieldFullDefenseAuto();
        //m_LeftMidfieldDefenseAuto = new LeftMidFieldDefenseAuto();
        //m_RightMidfieldDefenseAuto = new RighttMidFieldDefenseAuto();
        //m_ShootPreload = new ShootPreload(m_swerveSubsystem, m_shooterSubsystem, m_Trajectory);
        //m_ShootPreloadThenLMD = new ShootPreloadThenLMD();
        //m_ShootPreloadThenRMD = new ShootPreloadThenRMD();

        // Populate autoRoutineChooser and post it to Dashboard
        m_autoRoutineChooser.setDefaultOption("Do Nothing", m_DoNothingCmd);
        //m_autoRoutineChooser.addOption("Mid Full Defense", m_MidfieldFullDefenseAuto); //MFD
        //m_autoRoutineChooser.addOption("Left Mid Defense",m_LeftMidfieldDefenseAuto); //LMD
        //m_autoRoutineChooser.addOption("Right Mid Defense", m_RightMidfieldDefenseAuto); //RMD
        //m_autoRoutineChooser.addOption("Shoot Preload", m_ShootPreload);
        //m_autoRoutineChooser.addOption("Shoot Then MD", m_ShootPreloadThenLMD);
        //m_autoRoutineChooser.addOption("Shoot Then RMD", m_ShootPreloadThenRMD);
        SmartDashboard.putData(m_autoRoutineChooser);
        
        // Create subsystem default commands 
        m_swerveSubsystem.setDefaultCommand(
                new DefaultDriveCmd(m_swerveSubsystem,
                                    // Xbox stick forward and stick to left are both neg, 
                                    // so must negate all for WPILib expected coordinates:
                                    () -> -m_xbox.getLeftY(),    // Y = translate: +fore / -back
                                    () -> -m_xbox.getLeftX(),    // X = strafe: +left / -right
                                    () -> -m_xbox.getRightX())); // rotate: +CCW / -CW
        // Define and implement Drive Team UI
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
        //    ALT + Right bumper    => 
        //    b                     => Aim to April Tag
        //    ALT + b               => End Aim to April Tag
        //    y                     => Start shooting
        //    ALT + y               => Stop shooting, but leave flywheels going
        //    a                     => Extend intake
        //    ALT + a               => Retract intake (to hold position) 
        //    x                     => Stop spinning shooter (will coast to stop)
        //    ALT + x               => Swerve "park" cmd
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
        //    ALT + POV_UP          => Increments the shooter speed, in +200 RPM steps 
        //                              (3.33 rps steps), range 20 to 90 rps
        //    POV_DOWN              =>
        //    ALT + POV_DOWN        => Decrements the shooter speed, in -200 RPM steps
        //    POV_LEFT              => 
        //    ALT + POV_LEFT        => 
        //    POV_RIGHT             => 
        //    ALT + POV_RIGHT       =>
        //    ALT+LJoyStickButton   => 
        //    ALT+RJoystickButton   => 
        
        // The following methods can be used to trigger rotation about any given corner.
        // Most useful when playing defense, but also effective for demos

        // Front left corner
        m_xbox.leftTrigger().and(ALT.negate()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setFLCenOfRotation()));
        m_xbox.leftTrigger().and(ALT.negate()).onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));

        // Front right corner
        m_xbox.rightTrigger().and(ALT.negate()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setFRCenOfRotation()));                                        
        m_xbox.rightTrigger().and(ALT.negate()).onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation())); 

        // Back left corner
        ALT.and(m_xbox.leftTrigger()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setBLCenOfRotation()));
        ALT.and(m_xbox.leftTrigger()).onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        
        // Back right corner
        ALT.and(m_xbox.rightTrigger()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setBRCenOfRotation()));
        ALT.and(m_xbox.rightTrigger()).onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        
        // Left joystick button sets field oriented driving
        m_xbox.leftStick().and(ALT.negate()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setFieldOriented(true)));
        // Right joystick button sets robot oriented driving
        m_xbox.rightStick().and(ALT.negate()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setFieldOriented(false)));
        
        // back button zeros the Gyro
        m_xbox.back().and(ALT.negate()).onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro()));

        // Right bumper alone = slow mode (1/2 speed).
        m_xbox.rightBumper().and(ALT.negate()).onTrue(
                new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(.5)));
        // ALT + right bumper = super slow mode (1/5 speed)
        ALT.and(m_xbox.rightBumper()).onTrue(
                new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(.2)));
        // Right bumper release (regardless of ALT state), full speed.
        m_xbox.rightBumper().onFalse(
                new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(1.0)));

        // Extend intake
        // m_xbox.a().and(ALT.negate()).onTrue(new InstantCommand(()-> m_intakeSubsystem.arm_gotoFloorPickup()));
        // Retract intake (to hold position)
        // ALT.and(m_xbox.a()).onTrue(new InstantCommand(()-> m_intakeSubsystem.arm_gotoHold()));
        
        // y = Start Shooting
        m_xbox.y().and(ALT.negate()).onTrue(new InstantCommand(()-> m_shooterSubsystem.shootContinuous()));
        // ALT + y = Stop shooting
        ALT.and(m_xbox.y()).onTrue(new InstantCommand(()-> m_shooterSubsystem.stopShooting()));
        // x = Shut down shooter
        m_xbox.x().and(ALT.negate()).onTrue(new InstantCommand(()-> m_shooterSubsystem.shutdownShooter()));

        // ALT + x = Swerve park 
        ALT.and(m_xbox.x()).onTrue(m_parkCmd);

        // ALT + povUp = incrment shooter val
        ALT.and(m_xbox.povUp()).onTrue(new InstantCommand(()-> m_shooterSubsystem.incrementShooterVel()));
        // povUp = set shooter vel far
        m_xbox.povUp().and(ALT.negate()).onTrue(new InstantCommand(()-> m_shooterSubsystem.spinUpShooterFar()));
        // ALT + povDown = decremnt shooter vel
        ALT.and(m_xbox.povDown()).onTrue(new InstantCommand(()-> m_shooterSubsystem.decrementShooterVel()));
        // povDown = set shooter vel near
        m_xbox.povDown().and(ALT.negate()).onTrue(new InstantCommand(()-> m_shooterSubsystem.spinUpShooterClose()));
    }

    /*
     * getSelectedAutoCommand is called from Robot.AutonomousInit(),
     */
    public Command getSelectedAutoCommand() {
        return m_autoRoutineChooser.getSelected();
    }
}