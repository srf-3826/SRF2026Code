package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.turret.TurretModule;
import frc.lib.turret.TurretModuleConstants;
import frc.lib.geometry.Geometry.Vector3D;

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

    private Vector3D getTurretDimensions()
    {
        Vector3D turret = new Vector3D(
            TurretModuleConstants.TURRET_HEAD_CAMERA_FORWARD_OFFSET_M,
            TurretModuleConstants.TURRET_HEAD_CAMERA_SIDEWAYS_OFFSET_M,
            TurretModuleConstants.TURRET_HEAD_CAMERA_HEIGHT_M
        );
        return turret
            .rotRoll(0)
            .rotPitch(getPitch2d().getRadians())
            .rotYaw(getYaw2d().getRadians());
    }
    
    public double appliedCameraX() 
    {   
        Vector3D turretDimensions = getTurretDimensions();
        return 
            TurretModuleConstants.TURRET_BASE_FORWARD_OFFSET_M
            + turretDimensions.x;
    }
    public double appliedCameraY()
    {
        Vector3D turretDimensions = getTurretDimensions();
        return 
            TurretModuleConstants.TURRET_BASE_SIDEWAYS_OFFSET_M
            + turretDimensions.y;
    }
    public double appliedCameraHeight()
    {
        Vector3D turretDimensions = getTurretDimensions();
        return 
            TurretModuleConstants.TURRET_BASE_HEIGHT_M
            +turretDimensions.z;
    }
}
