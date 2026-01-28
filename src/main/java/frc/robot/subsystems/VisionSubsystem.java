package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers.LimelightResults;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase{
    
    private Integer errorCount = 0;
    private final SwerveSubsystem m_swerveSubsystem;
    public HashMap<String, Double> camData = new HashMap<String, Double>();

    //HashMap<String, Double> cam2Data = new HashMap<String, Double>();

    //PhotonCamera camera = new PhotonCamera("limelight");

   public VisionSubsystem(LimelightResults vision, SwerveSubsystem swerve){
        // TODO resolve why vision argument was passed in
        m_swerveSubsystem = swerve;
   } 
    
    
    public double[] getVisionData(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); //set getTable() to limelight name NOT pipeline name

    // check for target
        long Tv = (table.getEntry("tv").getInteger(0));
        camData.put("Tv", ((double)Tv));
        SmartDashboard.putNumber("Cam has targets", Tv);
        // SmartDashboard.getBoolean("Cam2 has targets", LimelightHelpers.getTV("cam2"));
    
    // get pipeline type
        String pipeType = LimelightHelpers.getCurrentPipelineType("limelight");
        SmartDashboard.putString("pipeType", pipeType);

    // get pipeline number
        double pipeNum = LimelightHelpers.getCurrentPipelineIndex("limelight");
        SmartDashboard.putNumber("pipeType", pipeNum);

    // get fid ID
        double fidID = LimelightHelpers.getFiducialID("limelight");
        SmartDashboard.putNumber("fidID", fidID);

    //get horizontal value
        double Tx = table.getEntry("tx").getDouble(0);
        camData.put("Tx", Tx);
        SmartDashboard.putNumber("TX", Tx);

    //get vertical value
        double Ty = table.getEntry("ty").getDouble(0);
        camData.put("Ty", Ty);
        SmartDashboard.putNumber("TY", Ty);

    //get area value
        double Ta = table.getEntry("ta").getDouble(0);
        camData.put("Ta", Ta);
        SmartDashboard.putNumber("TA", Ta);
            // tx,ty, and ta are used with 2d tracking

    // get targetpose_cameraspace data
        // this is relative to the target
        // recieved values are in meters
        var tempVar = table.getEntry("targetpose_cameraspace");
        double[] value = null;
        //System.out.println(tempVar);
        //table entries will never return null
        if( tempVar == null){
            ++errorCount;
            System.out.println("Target Pose Error: " + errorCount + "  " + System.currentTimeMillis());
        } else {
            //double[] lengthTempVar = tempVar.getDoubleArray(new double[6]); 
            //System.out.println("Array length: " + lengthTempVar.length);

            double[] targetpose = tempVar.getDoubleArray(new double[6]);

            double targetposeX = targetpose[0];
            double targetposeY = targetpose[1];
            double targetposeZ = targetpose[2];

            double targetposeRoll = targetpose[3];
            double targetposePitch = targetpose[4];
            double targetposeYaw = targetpose[5];
            // TODO: awkward to communicate through smart dashboard (i.e. NetworkTables). 
            // Is there a better alternative?
            camData.put("targetposeX", targetposeX);
            camData.put("targetposeY", targetposeY);
            camData.put("targetposeZ", targetposeZ);

            camData.put("targetposeRoll", targetposeRoll);
            camData.put("targetposePitch", targetposePitch);
            camData.put("targetposeYaw", targetposeYaw);

            SmartDashboard.putNumber("targetposeX", targetposeX);
            SmartDashboard.putNumber("targetposeY", targetposeY);
            SmartDashboard.putNumber("targetposeZ", targetposeZ);

            SmartDashboard.putNumber("targetposeRoll", targetposeRoll);
            SmartDashboard.putNumber("targetposePitch", targetposePitch);
            SmartDashboard.putNumber("targetposeYaw", targetposeYaw);
        
            SmartDashboard.putNumberArray("targetpose", targetpose ); 
            value = targetpose;
        } 
        return value;    
    }

    @Override
    public void periodic(){ 
        //Yaw = camera.getAllUnreadResults().get(0);
        //SmartDashboard.putBoolean("limelight Has Targets", camera.getLatestResult().hasTargets());
        getCamTrigDist();
    }

    public double getCamTrigDist(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        var Ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = Ty.getDouble(0.0);

        double angleToTargetRaidans = ( targetOffsetAngle_Vertical) * (3.14159 / 180.0);

        var aprilTagHeightMeters = Units.inchesToMeters(Constants.VC.APRILTAG_HEIGHT);
        var camHeight = Units.inchesToMeters(Constants.VC.CAM_HEIGHT);
        double distanceToTarget = (aprilTagHeightMeters - camHeight) / Math.tan(angleToTargetRaidans);

        camData.put("distanceTarget", distanceToTarget);
        SmartDashboard.putNumber("distTarget", distanceToTarget);  
        SmartDashboard.putNumber("Robot Gyro heading (deg)", m_swerveSubsystem.getYaw2d().getDegrees());  

        return distanceToTarget; 
    }
}