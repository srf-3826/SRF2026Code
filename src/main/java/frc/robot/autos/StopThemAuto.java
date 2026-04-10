/*package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SDC;
import frc.robot.subsystems.SwerveSubsystem;

public class StopThemAuto extends Command {
    SwerveSubsystem m_swerveSubsystem;
    PIDController m_drivePID;
    boolean finished = false;
    public StopThemAuto(SwerveSubsystem swerve) {
        m_swerveSubsystem = swerve;
        m_drivePID = new PIDController(1, 0, 0);
        m_drivePID.setSetpoint(5); // TODO: Find out how far to go.
        m_drivePID.setTolerance(0.1);
    }
    @Override
    public void execute() {
        Translation2d position = m_swerveSubsystem.getPose().getTranslation();
        
        if (m_drivePID.atSetpoint()) {
            finished = true;
        } else {
            m_swerveSubsystem.drive(
                new Translation2d(m_drivePID.calculate(position.getX()), 0), 
                0, 
                true);
        }
    }
    @Override
    public void end(boolean interrupted) {
        double [] directions = {
            SDC.PARK_ANGLE_LEFT_DEG,
            SDC.PARK_ANGLE_RIGHT_DEG,
            -SDC.PARK_ANGLE_LEFT_DEG,
            -SDC.PARK_ANGLE_RIGHT_DEG
        };
        m_swerveSubsystem.rotateModulesToAngles(directions);
    }

}
*/