// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

// import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
// import frc.robot.commands.RumbleCmd;
// import frc.robot.commands.WaitForMillisecsCmd;
import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;

// The Auto Cmd is intended to create a simulated path for the robot to
// follow to pick up a note from the field, then return to the speaker goal
// and score that note. The path should be short enough to test in the lab,
// with twists and turns in both segments to make the test challenging.
public class AutoPathTestCmd extends SequentialCommandGroup {
    SwerveSubsystem m_swerve;
    Trajectory m_trajectory;

    public AutoPathTestCmd(SwerveSubsystem swerve, Trajectory calculatedTrajectory) {
        m_swerve = swerve;
        m_trajectory = calculatedTrajectory;
        
        TrajectoryConfig config =
            new TrajectoryConfig(AutoC.AUTO_MAX_SPEED_M_PER_SEC *
                                    AutoC.AUTO_SPEED_FACTOR_GENERIC,
                                AutoC.AUTO_MAX_ACCEL_M_PER_SEC2 *
                                    AutoC.AUTO_ACCEL_FACTOR_GENERIC)
                .setKinematics(SDC.SWERVE_KINEMATICS);
                // .addConstraint(AutoConstants.autoVoltageConstraint);
        config.setReversed(false);

        ProfiledPIDController thetaController =
                                new ProfiledPIDController(AutoC.KP_THETA_CONTROLLER,
                                                          AutoC.KI_THETA_CONTROLLER,
                                                          0,
                                                          AutoC.K_THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // A trajectory (in this case passed in as an argument) is composed of 3 separate parts
        // at a minimum - a start point (and heading), waypoints, and a stop point (and heading).
        // All units in meters.

        SwerveControllerCommand autoPathTestCmd =
                new SwerveControllerCommand(
                    m_trajectory,
                    m_swerve::getPose,
                    SDC.SWERVE_KINEMATICS,
                    new PIDController(AutoC.KP_X_CONTROLLER, AutoC.KI_X_CONTROLLER, 0),
                    new PIDController(AutoC.KP_Y_CONTROLLER, 0, 0),
                    thetaController,
                    m_swerve::setModuleStates,
                    m_swerve);

        addCommands(new InstantCommand(() -> m_swerve.resetOdometry(m_trajectory.getInitialPose())),
                      autoPathTestCmd,
                
                    new InstantCommand(()-> m_swerve.stop()));
    }
}