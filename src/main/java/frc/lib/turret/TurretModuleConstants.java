package frc.lib.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class TurretModuleConstants {
    public final int STEER_MOTOR_ID;
    public final int ENCODER_ID;
    public final Rotation2d ABS_ANG_OFFSET2D;
    public final ShuffleboardLayout SBE_LAYOUT;

    public static double TURRET_ROTATION_TO_DEGREES = 1;

    public static double TURRET_BASE_HEIGHT_M = 0; // Turret base height in meters
    public static double TURRET_BASE_FORWARD_OFFSET_M = 0;
    public static double TURRET_BASE_SIDEWAYS_OFFSET_M = 0;

    public static double TURRET_HEAD_CAMERA_HEIGHT_M = 0; // the total camera height, minus Turret base height. The distance from the base to the camera.
    public static double TURRET_HEAD_CAMERA_FORWARD_OFFSET_M = 0;
    public static double TURRET_HEAD_CAMERA_SIDEWAYS_OFFSET_M = 0;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * Setup variables in, per module, stored as module running constants.
     * @param driveMotorId
     * @param steerMotorId
     * @param encoderId
     * @param absEncoderOffset2d
     * @param sBE_Layout           (ShuffleBoard Layout handle - to hold SB entry keys)
     */
    public TurretModuleConstants(int steerMotorID, 
                                 int canCoderID, 
                                 Rotation2d absEncoderOffset2d,
                                 ShuffleboardLayout sBE_Layout) {
        STEER_MOTOR_ID = steerMotorID;
        ENCODER_ID = canCoderID;
        ABS_ANG_OFFSET2D = absEncoderOffset2d;
        SBE_LAYOUT = sBE_Layout;
    }
}
