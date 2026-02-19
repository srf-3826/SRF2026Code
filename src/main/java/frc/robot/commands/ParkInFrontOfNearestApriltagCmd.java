package frc.robot.commands;

import java.util.function.DoubleSupplier;

import javax.sound.midi.Track;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SDC;
import frc.robot.Constants.UIC;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.lib.geometry.Geometry.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

public class ParkInFrontOfNearestApriltagCmd extends Command {
    private SwerveSubsystem m_swerveSubsystem; 
    private VisionSubsystem m_visionSubsystem;
    
    private DoubleSupplier m_translationSup;
    private DoubleSupplier m_strafeSup;
    private DoubleSupplier m_rotationSup;

    private double[] cameraData;

    private Pose3d targetData;

    private boolean shouldGo;

    public ParkInFrontOfNearestApriltagCmd(
        SwerveSubsystem swerveSubsystem,
        VisionSubsystem visionSubsystem,
        DoubleSupplier translationSup, 
        DoubleSupplier strafeSup, 
        DoubleSupplier rotationSup
    )
    {

        m_swerveSubsystem = swerveSubsystem;
        m_visionSubsystem = visionSubsystem;
        addRequirements(swerveSubsystem, visionSubsystem);

        m_translationSup = translationSup;
        m_strafeSup = strafeSup;
        m_rotationSup = rotationSup;
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        int[] ids = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
        m_visionSubsystem.setTargetFilter(ids);
        locationSolverX = new PIDController(0.2, 0, 0);
        locationSolverX.setTolerance(0.1);
        locationSolverY = new PIDController(0.2, 0, 0);
        locationSolverY.setTolerance(0.1);
    }

    PIDController locationSolverX;
    PIDController locationSolverY;
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_visionSubsystem.tryLocateBot()) {
            targetData = m_visionSubsystem.getNearestTarget();
            Translation3d targetLoc = targetData.getTranslation();
            Rotation3d targetFace = targetData.getRotation();

            Translation2d targetLoc2d = new Translation2d(targetLoc.getX(), targetLoc.getY());
            Translation2d absoluteTargetLoc2d = targetLoc2d.plus(m_visionSubsystem.poses.getBotPose().getTranslation());

            Translation2d angledOffset = new Translation2d(1*Math.cos(targetFace.getZ()), 1*Math.sin(targetFace.getZ()));

            double x = targetLoc2d.getX()+angledOffset.getX();
            double y = targetLoc2d.getY()+angledOffset.getY();
            locationSolverX.setSetpoint(x);
            locationSolverY.setSetpoint(y);

            Translation2d power = new Translation2d(
                locationSolverX.calculate(x),
                locationSolverY.calculate(y)
            );
            m_swerveSubsystem.drive(power, 0, true);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // End when any joystick input becomes active. Note it is left to
        // the user via DefaultDriveCmd to set the next module directions;
        // the modules will stay "parked" with motors off until then.
        if ((Math.abs(m_translationSup.getAsDouble()) > UIC.JOYSTICK_DEADBAND)
                ||
                (Math.abs(m_strafeSup.getAsDouble()) > UIC.JOYSTICK_DEADBAND)
                ||
                (Math.abs(m_rotationSup.getAsDouble()) > UIC.JOYSTICK_DEADBAND))
        {
            return true;
        }
        if (cameraData.length == 0) {
            return true;
        }
        return false;
    }
}
