package frc. robot.subsystems;
/*
import static edu.wpi.first.math.util.Units.degreesToRadians;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;

import frc.robot.Constants.GC;
import frc.robot.Constants.SDC;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.trajectory.Trajectory;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;



public class PoseEstimatorSubsystem extends SubsystemBase {


  //private final SmartDashboard 
  private final PhotonCamera limelight;
  private final SwerveSubsystem m_swerveSubsystem;
  private final Transform3d CAMERA_TO_ROBOT;
  private final Pigeon2 m_gyro;
  private final Rotation2d getRotation2d;
  // Ordered list of target poses by ID (WPILib is adding some functionality for
  // this)
  private static final List<Pose3d> targetPoses = Collections.unmodifiableList(List.of(
      new Pose3d(3.0, 1.165, 0.287 + 0.165, new Rotation3d(0, 0, degreesToRadians(180.0))),
      new Pose3d(3.0, 0.0, 0.287 + .165, new Rotation3d(0, 0, degreesToRadians(180.0)))));
  
  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta, s_0, ... s_n]ᵀ, with units in meters and radians, then meters.
   */
  /*
  private static final Vector<N7> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05, 0.05, 0.05);
  
  /**
   * Standard deviations of the encoder and gyro measurements. Increase these numbers to trust sensor readings from
   * encoders and gyros less. This matrix is in the form [theta, s_0, ... s_n], with units in radians followed by meters.
   */
  //private static final Vector<N5> localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.01), 0.01, 0.01, 0.01, 0.01);
  
  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   *//*
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  //private double previousPipelineTimestamp = 0;

  private final Pose2d initialPoseMeters;
  
  public PoseEstimatorSubsystem(PhotonCamera limelight, SwerveSubsystem swerveSubsystem) {
    this.limelight = limelight;
    m_swerveSubsystem = swerveSubsystem;

    initialPoseMeters = m_swerveSubsystem.getPose();

    poseEstimator =  new SwerveDrivePoseEstimator(
        SDC.SWERVE_KINEMATICS,
        m_swerveSubsystem.getYaw2d(),
        m_swerveSubsystem.getModulePositions(),
        initialPoseMeters);
    
    CAMERA_TO_ROBOT = new Transform3d();
    getRotation2d = new Rotation2d();
        m_gyro = new Pigeon2(GC.PIGEON_2_CANID, Constants.ROBO_RIO_BUS_NAME);


  }*/

  //@Override
  /*public void periodic() {
    // Update pose estimator with the best visible target
    var pipelineResult = limelight.getLatestResult();
    var resultTimestamp = pipelineResult.getTimestampSeconds();

    SmartDashboard.putBoolean("Camera Has Targets", limelight.getLatestResult().hasTargets() == true);

    if (/*resultTimestamp != previousPipelineTimestamp && *///pipelineResult.hasTargets()) {
      //previousPipelineTimestamp = resultTimestamp;
      /*var target = pipelineResult.getBestTarget();
      var fiducialId = target.getFiducialId();
      if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && fiducialId < targetPoses.size()) {
        var targetPose = targetPoses.get(fiducialId);
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
        poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);

      }
    
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
      m_swerveSubsystem.getYaw2d(),
      m_swerveSubsystem.getModulePositions());

    field2d.setRobotPose(getCurrentPose());
    }


    
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        m_swerveSubsystem.getYaw2d().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }
*/
  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   *//*
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
      m_swerveSubsystem.getYaw2d(),
      m_swerveSubsystem.getModulePositions(),
      newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
 /* public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

}*/             
