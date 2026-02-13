package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.lib.turret.TurretModule;
import frc.lib.turret.TurretModuleConstants;
import edu.wpi.first.math.geometry.Pose3d;

public class TurretSubsystem {
    public TurretModule turretBase = new TurretModule(0, null, null);

    public Rotation2d getYaw2d()
    {
        Rotation2d yaw = turretBase.getAngle2d();
        return yaw;
    }
    public Rotation2d getPitch2d()
    {
        
        return new Rotation2d();
    }

    public Pose3d getTurretDimensions()
    {
        Pose3d turret = new Pose3d(
            TurretModuleConstants.TURRET_HEAD_CAMERA_FORWARD_OFFSET_M,
            TurretModuleConstants.TURRET_HEAD_CAMERA_SIDEWAYS_OFFSET_M,
            TurretModuleConstants.TURRET_HEAD_CAMERA_HEIGHT_M,
            new Rotation3d(
                0, 
                getPitch2d().getDegrees(),
                getYaw2d().getDegrees()
            )
        );
        return turret;
    }
    
    public double appliedCameraX() 
    {   
        Pose3d turretDimensions = getTurretDimensions();
        return 
            TurretModuleConstants.TURRET_BASE_FORWARD_OFFSET_M
            + turretDimensions.getX();
    }
    public double appliedCameraY()
    {
        Pose3d turretDimensions = getTurretDimensions();
        return 
            TurretModuleConstants.TURRET_BASE_SIDEWAYS_OFFSET_M
            + turretDimensions.getY();
    }
    public double appliedCameraHeight()
    {
        Pose3d turretDimensions = getTurretDimensions();
        return 
            TurretModuleConstants.TURRET_BASE_HEIGHT_M
            +turretDimensions.getZ();
    }
}
