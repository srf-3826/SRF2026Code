package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;
import frc.lib.geometry.Geometry.Vector3D;
import frc.lib.geometry.Geometry.Point3D;

public class TrackingSubsystem extends SubsystemBase {


    private  SwerveSubsystem m_drivetrain;
    

    private TurretSubsystem m_turret;


    public TrackingSubsystem(SwerveSubsystem drivetrain, TurretSubsystem turret)
    {
        this.m_drivetrain = drivetrain;
        this.m_turret = turret;
    }
    private double distanceBetween(Vector3D a, Vector3D b)
    {
        return Math.sqrt((Math.pow(b.x-a.x,2)+Math.pow(b.y-a.y, 2))+Math.pow(b.z-a.z,2));
    }
    

    Point3D[] targets = {};

    public Vector3D cameraToRobot(Vector3D point)
    {
        // Correct for the robot's viewing angle.
        point = point
            .rotRoll(0)
            .rotPitch(m_turret.getPitch2d().getRadians())
            .rotYaw(m_drivetrain.getYaw2d().getRadians()+m_turret.getYaw2d().getRadians());

        point.x += m_turret.appliedCameraX();
        point.y += m_turret.appliedCameraY();
        point.z += m_turret.appliedCameraHeight();

        return point;
    }
    public Vector3D cameraToWorld(Vector3D point)
    {
        point = cameraToRobot(point);
        // TODO: Verify this is correct
        Pose2d position = m_drivetrain.getPose();
        point.x += position.getX();
        point.y += position.getY();
        return point;
    }
    public void calibrateOdometry()
    { // TODO: Take known location of tag and location given by camera to robot, then offset odometry to correct for it.
        
    }
}
