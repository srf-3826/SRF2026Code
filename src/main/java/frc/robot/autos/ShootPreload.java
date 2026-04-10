/*package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoC;
import frc.robot.Constants.SDC;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.commands.*;
import java.util.List;

public class ShootPreload extends SequentialCommandGroup {

    ShooterSubsystem m_ShooterSubsystem;
    SwerveSubsystem m_SwerveSubsystem;
    Trajectory m_Trajectory;
    WaitForMillisecsCmd m_WaitForMillisecsCmd;
    
    public ShootPreload(SwerveSubsystem swerve, ShooterSubsystem shooter, Trajectory calculatedTrajectory){
        m_SwerveSubsystem = swerve;
        m_ShooterSubsystem = shooter;
        m_Trajectory = calculatedTrajectory;
        m_WaitForMillisecsCmd = new WaitForMillisecsCmd(2000);
    
        TrajectoryConfig config = 
        new TrajectoryConfig(AutoC.AUTO_MAX_SPEED_M_PER_SEC *
                                    AutoC.AUTO_SPEED_FACTOR_GENERIC,
                                AutoC.AUTO_MAX_ACCEL_M_PER_SEC2 *
                                    AutoC.AUTO_ACCEL_FACTOR_GENERIC)
                .setKinematics(SDC.SWERVE_KINEMATICS);
                config.setReversed(false);

        ProfiledPIDController thetaController =
                                new ProfiledPIDController(AutoC.KP_THETA_CONTROLLER,
                                                          AutoC.KI_THETA_CONTROLLER,
                                                          0,
                                                          AutoC.K_THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory LeaveHub = 
        TrajectoryGenerator.generateTrajectory(new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
        List.of(new Translation2d(0.0,1.0)),
        new Pose2d(0.0, 2.0, new Rotation2d(0)),
        config);

        SwerveControllerCommand driveFromTheHubToShoot =
                new SwerveControllerCommand(
                    LeaveHub,
                    m_SwerveSubsystem::getPose,
                    SDC.SWERVE_KINEMATICS,
                    new PIDController(AutoC.KP_X_CONTROLLER, AutoC.KI_X_CONTROLLER, 0),
                    new PIDController(AutoC.KP_Y_CONTROLLER, 0, 0),
                    thetaController,
                    m_SwerveSubsystem::setModuleStates,
                    m_SwerveSubsystem);

        addCommands(
            new InstantCommand(()-> m_ShooterSubsystem.spinUpFlywheelClose()),
            new InstantCommand(() -> m_SwerveSubsystem.resetOdometry(LeaveHub.getInitialPose())),
            driveFromTheHubToShoot,
            new InstantCommand(()-> m_ShooterSubsystem.shootContinuous()),
            m_WaitForMillisecsCmd,
            new InstantCommand(()-> m_ShooterSubsystem.stopShooting())
         );
    }
}
*/