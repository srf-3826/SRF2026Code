package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.sensors.GyroIO;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private CANBus swerveCanbus = new CANBus(Constants.CAN_BUS_FOR_SWERVE);                // Still needed for Gryo!
    private CANBus allElseCanbus = new CANBus(Constants.CAN_BUS_FOR_EVERYTHING_ELSE);

    // Declare subsystem object handles
    private GyroIO                 m_gyroIO;
    private IntakeSubsystem        m_intakeSubsystem;
    private ShooterSubsystem       m_shooterSubsystem;

    // Declare commands needed for Teleop and Auto
    //private DeployIntakeCmd             m_DeployIntakeCmd;;
    //private RetractIntakeCmd            m_RetractIntakeCmd;
    //private EjectIntakeCmd              m_EjectIntakeCmd;

    private DoNothingCmd                m_DoNothingCmd;
 
    // Create sendable chooser for desired Auto routine
    private static SendableChooser<Command> m_autoRoutineChooser = new SendableChooser<>();

    // Declare CommandXboxController
    private static CommandXboxController m_xbox;

    //  Constructor for the robot container. Contains subsystems, OI devices, and commands.
    public RobotContainer() {
        m_xbox =             new CommandXboxController(0);
        m_gyroIO =           new GyroIO(GC.PIGEON_2_CANID, GC.INVERT_GYRO, swerveCanbus);
        m_intakeSubsystem =  new IntakeSubsystem(allElseCanbus);
        m_shooterSubsystem = new ShooterSubsystem(allElseCanbus, m_intakeSubsystem);

        // Create commands used for teleop
        //m_RetractIntakeCmd = new RetractIntakeCmd(m_intakeSubsystem);
        //m_DeployIntakeCmd = new DeployIntakeCmd(m_intakeSubsystem);
        //m_EjectIntakeCmd = new EjectIntakeCmd(m_intakeSubsystem);

        // Create commands used for Auto
        m_DoNothingCmd =     new DoNothingCmd();

        // Populate autoRoutineChooser and post it to Dashboard
        m_autoRoutineChooser.setDefaultOption("Do Nothing", m_DoNothingCmd);
        SmartDashboard.putData(m_autoRoutineChooser);
        
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
        //    Right Bumper          => 
        //    ALT + Right bumper    => 
        //    b                     => 
        //    ALT + b               => 
        //    y                     => Start shooting
        //    ALT + y               => Stop shooting, but leave flywheels going
        //    a                     => Extend intake
        //    ALT + a               => Retract intake (to hold position) 
        //    x                     => Stop spinning shooter (will coast to stop)
        //    ALT + x               => 
        //    RightTrigger          => 
        //    ALT + RightTrigger    => 
        //    LefTrigger            => 
        //    ALT + LeftTrigger     => 
        //    L Joystick Y          => 
        //    L Joystick X          => 
        //    R Joystick X          => 
        //    R Joystick Y          =>
        //    L Joystick Button     => 
        //    R Joystick Button     => 
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
        
        // back button zeros the Gyro
        m_xbox.back().and(ALT.negate()).onTrue(new InstantCommand(() -> m_gyroIO.zeroGyro()));

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