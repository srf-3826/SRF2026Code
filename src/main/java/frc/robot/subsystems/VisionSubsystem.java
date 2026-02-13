package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Constants.VC;
import frc.robot.subsystems.TurretSubsystem;

public class VisionSubsystem extends SubsystemBase {
    LimelightResults m_limelightResults;
    SwerveDrivePoseEstimator m_poseEstimator;
    SwerveSubsystem m_drivetrain;
    LimelightHelpers.IMUData imuData;
    TurretSubsystem m_turret;
    public PoseClass poses;
    
    public VisionSubsystem(SwerveDrivePoseEstimator poseEstimator, SwerveSubsystem drivetrain, TurretSubsystem turret)
    {
        m_limelightResults = new LimelightResults();
        m_poseEstimator = poseEstimator;
        m_drivetrain = drivetrain;
        m_turret = turret;
        imuData = new LimelightHelpers.IMUData();
        poses = new PoseClass();

        Pose3d camPose = m_turret.getTurretDimensions();
        setCameraPos(
            camPose,
            camPose.getRotation()
        );
    }

    public void setCameraPos(Pose3d position, Rotation3d rotation)
    {
        LimelightHelpers.setCameraPose_RobotSpace(
            VC.LIMELIGHT_NAME,

            position.getX(), // back to front
            position.getY(), // right to left
            position.getZ(), // down to up

            rotation.getX(), // Roll
            rotation.getY(), // Pitch
            rotation.getZ() // Yaw
        );
    }
    @Override
    public void periodic() 
    {
        LimelightHelpers.SetRobotOrientation(
            VC.LIMELIGHT_NAME,
            m_drivetrain.getYaw2d().getDegrees(), 0,
            0, 0,
            0, 0);
            m_poseEstimator.update(m_drivetrain.getYaw2d(), m_drivetrain.getModulePositions());
    }
    public boolean tryLocateBot()
    { // returns tru on success, false on fail
        LimelightHelpers.SetRobotOrientation(VC.LIMELIGHT_NAME, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VC.LIMELIGHT_NAME);
        boolean doRejectUpdate = false;
        imuData = LimelightHelpers.getIMUData(VC.LIMELIGHT_NAME);
        
        // if our angular velocity is greater than 360 degrees per second, ignore vision updates
        // TODO: find out what that translates to. This is incomplete
        /*
        if(Math.abs(imuData.accelX) > 360)
        {
            doRejectUpdate = true;
        }
        */
        if(megatag2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {   // TODO: These numbers are magic to me. I'm not sure what they do.
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999)); 
            m_poseEstimator.addVisionMeasurement(
                megatag2.pose,
                megatag2.timestampSeconds);
        }
        return !doRejectUpdate;
    }
    public void setTargetFilter(int[] ids)
    {
        LimelightHelpers.SetFiducialIDFiltersOverride(VC.LIMELIGHT_NAME, ids);
    }
    /**
     * use getRotation and getTranslation when using this value to avoid unexpected issues
     * @return Pose3d
     */
    public Pose3d getNearestTarget()
    {
        double[] targetData = LimelightHelpers.getTargetPose_RobotSpace(VC.LIMELIGHT_NAME);
        return new Pose3d(targetData[0],targetData[1],targetData[2], new Rotation3d(targetData[3],targetData[4],targetData[5]));
    }
    public class PoseClass
    {
        public Pose2d getBotPose()
        {
            return m_poseEstimator.getEstimatedPosition();
        }
    }
}
