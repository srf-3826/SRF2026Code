package frc.lib.swerve;
//import frc.robot.Constants;
import frc.robot.Constants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
//import edu.wpi.first.units.measure.Angle;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
//import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

/*import com.revrobotics.spark.SparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
*/


public class SwerveModule {
    public  int m_modNum;
    private SwerveModuleConstants m_moduleConstants;
    private double m_absAngleOffsetRot;
    
    private double desiredAngle;      // Just a local variable, persistent to relieve Java GC pressure
    private double m_lastAngle;       // This is not used for control, rather it 
                                      // stores the last setpoint requested for a
                                      // given module, for data publishing purposes.
    private final TalonFX m_steerMotor;
    // private final RelativeEncoder m_integratedSteerEncoder;
    // private final SparkClosedLoopController m_steerController;
    private final CANcoder m_absWheelAngleCANcoder;

    private final TalonFX m_driveMotor;
/* 
    private SparkBaseConfig m_sparkBaseConfig;
    private ResetMode m_sparkResetMode;
    private SparkMaxConfig m_sparkMaxConfig = new SparkMaxConfig();
    private ClosedLoopConfig m_closedLoopConfig = new ClosedLoopConfig();
*/
    // Declare Phoenix6 control request objects for the Drive Motor:
    // Open loop control output to the drive motor is one shot DutyCycle, and must be 
    // repeated every loop to avoid safety timeout
    final DutyCycleOut m_driveOpenLoop = new DutyCycleOut(0.0).withUpdateFreqHz(0);
    // In phoenix5 the closed loop drive control was voltage compensated with arbitrary feed forward.
    // In Phoenix6 the equivalent is VelocityVoltage control, again with arbitrary feed forward calculated
    // per loop. The default update rate is 100 Hz, leave it as is and bump it every loop with updated
    // feed forward calculations.
    final VelocityVoltage m_driveClosedLoop = new VelocityVoltage(0.0);
    final PositionVoltage m_steerClosedLoop = new PositionVoltage(0.0);

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SDC.DRIVE_KS,
                                                                            SDC.DRIVE_KV,
                                                                            SDC.DRIVE_KA);
    private double m_velocityFeedForward;
    //private double m_positionFeedForward;


    // Entries for publishing module data.
    // Technically, the following member Entry variables do not need to be saved,
    // as the data written via them is static and should never change after
    // initial setup, i.e. they only need to be published once.
    // private GenericEntry IdsEntry;       // Drive ID, Rotate ID, CANcoder ID, in that order
                                            // Single digits only, to fit module list width
    // private GenericEntry absOffsetEntry;

    // However, the following Entry keys are for dynamic variables, and
    // will change depending on activity, so need publishing every loop.
    private GenericEntry steerSetpointDegEntry;
    private GenericEntry steerEncoderDegEntry;
    private GenericEntry absCANcoderDegEntry;
    private GenericEntry steerPIDOutputEntry;
    private GenericEntry wheelCurrPosEntry;
    private GenericEntry wheelCurrSpeedEntry;
    private GenericEntry wheelAmpsEntry;
    private GenericEntry wheelTempEntry;
    private GenericEntry steerAmpsEntry;
    private GenericEntry steerTempEntry;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants, CANBus swerveCanbus) {

        m_modNum = moduleNumber;
        m_moduleConstants = moduleConstants;
        m_absAngleOffsetRot = moduleConstants.ABS_ANG_OFFSET2D.getDegrees() / 360.0;

        /* Angle Motor Config */
        m_steerMotor = new TalonFX(m_moduleConstants.STEER_MOTOR_ID, swerveCanbus);
        configSteerMotor();

        /* Drive Motor Config */
        m_driveMotor = new TalonFX(m_moduleConstants.DRIVE_MOTOR_ID, swerveCanbus);
        configDriveMotor();

        /* Angle Encoder Config */
        m_absWheelAngleCANcoder = new CANcoder(m_moduleConstants.ENCODER_ID, swerveCanbus);

        m_lastAngle = getState().angle.getDegrees();

        setupModulePublishing();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Below is a custom optimize function, since default WPILib optimize assumes 
        // continuous controller which CTRE and Rev onboard were not.
        // The job of optimize is to minimise the amount of rotation required to
        // get to the new direction, given the actual current direction, of the
        // swerve module.
        desiredState = SwerveOptimize.optimize(desiredState, getState().angle);
        desiredAngle = desiredState.angle.getDegrees();
        // But, (for normal operation, not testing of a single module) only 
        // rotate the module if wheel speed is more than 1% of robot's max speed. 
        // This prevents Jittering. Otherwise, no turn needed, as closed loop
        // steering will maintain prior direction.
        if (Math.abs(desiredState.speedMetersPerSecond) > (SDC.MAX_ROBOT_SPEED_M_PER_SEC * 0.01)) {
            setAngle(desiredAngle);
        }
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(isOpenLoop) {
            // Drive using joystick. Convert MPS to a DutyCycle (was percent) output
            m_driveOpenLoop.Output = desiredState.speedMetersPerSecond / SDC.MAX_ROBOT_SPEED_M_PER_SEC;
            m_driveMotor.setControl(m_driveOpenLoop);
        }
        else {
            // Drive using VelocityVoltage PID, using Falcon encoder units and default Slot0
            m_driveClosedLoop.Velocity = desiredState.speedMetersPerSecond * SDC.MPS_TO_TALONFX_RPS_FACTOR;
            m_velocityFeedForward = feedforward.calculate(desiredState.speedMetersPerSecond);
            m_driveMotor.setControl(m_driveClosedLoop.withFeedForward(m_velocityFeedForward));
        }
    }

    public void setAngle(double desiredAngle) {
        m_steerClosedLoop.Position = desiredAngle * SDC.ANGLE_TO_ROTATION_FACTOR;
        m_steerMotor.setControl(m_steerClosedLoop);
        m_lastAngle = desiredAngle;
    }

    public void testDriveMotorRotation() {
        // this routime spins the drive motor slowly under percent output
        // to check that it is configured correctly (CCW Positive)
        m_driveMotor.setControl(m_driveOpenLoop
                                .withUpdateFreqHz(50)
                                .withOutput(0.25));
    }

    public void testSteerMotorRotation() {
        // this routime spins the steering motor slowly under percent output
        // to check that it is configured correctly (CCW Positive)
        m_steerMotor.setControl(m_driveOpenLoop
                                .withUpdateFreqHz(50)
                                .withOutput(0.25));
    }

    // getAngle2d returns the current swerve module direction as a Rotation2d value.
    private Rotation2d getAngle2d() {
        return convertToRotation2d(m_steerMotor.getPosition().getValueAsDouble() * 360.0);
    }

    private Rotation2d convertToRotation2d(double angleInDegrees) {
        // Convert the angle from degrees to radians and create a Rotation2d object
        return new Rotation2d(Math.toRadians(angleInDegrees));
    }

    // setFalconPosDeg is called from resetToAbsolute. The Falcon encoder is now configured to 
    // use native units (deg), so the angle argument must be in units of degrees (range 0-360)
    // reflecting the actual module direction (relative to the robot) upon initialization.
    // With the module aimed straight ahead, bevel gear to the left, the angle should be 0.0
    public void setFalconPosDeg(double angle) {
        m_steerMotor.setPosition(angle * SDC.ANGLE_TO_ROTATION_FACTOR);
    }

    // getFalconPosDeg returns the current value of the Falcon's integrated encoder (initialized 
    // at startup to the module's correct absolute direction) in degrees, in the range 0 to 360.
    public double getFalconPosDeg() {
        return normalizeAngle0To360(m_steerMotor.getPosition().getValueAsDouble());
    }

    public double normalizeAngle0To360(double angle) {
        // There are many ways to normalize an angle to the range 0 to 360 degrees with 
        // robustness for all possible negative and positive values, and we need to do
        // that for the integrated encoder in the NEO because while the PID is configured to
        // treat it as a "continuous" sensor (0 to 360) the sensor itself just continues to
        // linearly increase or decrease the position value per the rotation direction.
        // For keeping track of module heading it helps to bring it back to baseband, i.e.
        // to the range 0 - 360.
        // Note that for module steering PID purposes in this swerve drive code, native
        // degrees are used.
        // However, for most WPILib swerve support APIs, angles are expected to be radians 
        // in the range -PI to PI, using Rotation2d. This is not a problem as the Rotation2d 
        // object provides utility methods to convert degrees to radians and vice versa.
        // Math.IEEEremainder(angle, 360.0) will also normalize an angle;
        return ((angle % 360) + 360) % 360;
    }

    public double getRawFalconPos() {
        return m_steerMotor.getPosition().getValueAsDouble();
    }

    public double getCANcoderDeg() {
        // .getAbsolutePosition returns a StatusSignal, so we need to get it's value 
        // as a double and convert its range ([0, 1] rotations) into degrees
        return m_absWheelAngleCANcoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    }

    public Rotation2d getCANcoder2d(){
        return Rotation2d.fromDegrees(getCANcoderDeg());
    }

    private void waitForCANcoder() {
        // Wait for up a good CANcoder signal. This prevents a race condition during program startup
        // where trying to synchronize the Integrated motor encoder to the CANcoder before we have
        // received any position signal from the CANcoder results in failure.
        var posStatus = m_absWheelAngleCANcoder.getAbsolutePosition();

        for (int i = 0; i < 100; ++i) {
            if (posStatus.getStatus().isOK()) {
                break;
            } else {
                // Wait 10 ms and try again
                Timer.delay(0.010);    // Only called at initialization, or rarely from
                                               // teleop where no boot up delays are expected, 
                                               // so .delay() is ugly here but OK to use.
                posStatus = m_absWheelAngleCANcoder.getAbsolutePosition();
            }
            SmartDashboard.putNumber("Mod"+m_modNum+"AbsCANcoder read failed, read tries = ", i);         
        }
        posStatus.waitForUpdate(200);   // Then wait up to 200 ms for one more refresh before returning
    }

    // NOTE: Module steering motor built in encoders are used to track
    // and control wheel angles via thir built in PID functions.
    // At startup the wheels could be in any random orientation,
    // so absolute encoders are used to "seed" each steering motor encoder.
    // The absolute encoders (CANCoders) are initialized prior to this
    // method being called, and should have their MagetOffsets set to correct
    // each to return correct angles, i.e. they are
    // calibrated with their respective wheel absolute offset.
    // NOTE: recallibration requires a power cycle reset after any
    // MagnetOffset value is written, whether 0 (before measuring an offset)
    // or a measured offset value. Offsets should always be the negated reading
    // obtained with wheels straight ahead, wuth an existing offset of zero. 
    // Setting the motor encoder to match the absolute angle read by the 
    // CANCoder could be done at anytime, with the wheels at any angle, but 
    // it should only need to be done once at startup, barring any power glitches.
    public void resetToAbsolute(){
        waitForCANcoder();
        double CANcoderOnReset = getCANcoderDeg();
        //double absModuleDegOnReset = CANcoderOnReset - m_absAngleOffset2d.getDegrees();
        //SmartDashboard.putString("Mod"+m_modNum+" CANcoder on Reset", F.df2.format(CANcoderOnReset));
        setFalconPosDeg(CANcoderOnReset);
    }

    private void configAbsWheelAngleCANcoder() { 
        var magnetSensorConfigs = new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(SDC.CANCODER_RANGE)
                                                             .withSensorDirection(SDC.CANCODER_DIR)
                                                             .withMagnetOffset(m_absAngleOffsetRot);
        var ccConfig = new CANcoderConfiguration().withMagnetSensor(magnetSensorConfigs);
        m_absWheelAngleCANcoder.getConfigurator().apply(ccConfig);
        // NOTE: the MagnetOffset part of this config does NOT take effect until the next reboot.
        // But as long as we are writing the correct calibrated offset each time, it won't matter.
    }

/*
     // call after Absolute offsets have been set
     public void slowCanCoderBusUsage() {
        // Don't use module CANCoders at all after initialization, so this is provided 
        // to slow Cancoder usage way down
        m_absWheelAngleCANcoder.getPosition().setUpdateFrequency(5);
        m_absWheelAngleCANcoder.getVelocity().setUpdateFrequency(5);
        // could call optimizeBus(), but that may affect logging.
     }

     // Wait for success before return? 
     // Or call some time before needing encoders again (e.g. to RE-set absolute offsets)?
     public void speedUpCancoderBusReports() {
        m_absWheelAngleCANcoder.getPosition().setUpdateFrequency(100);
     }

    private void reportRevError(REVLibError errorCode) {
        if (errorCode != REVLibError.kOk) {
            SmartDashboard.putString("Mod "+m_modNum+"RevLibError = ", errorCode.toString());
        }
    }
*/
    private void configSteerMotor() {
        var openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                       .withVoltageOpenLoopRampPeriod(SDC.OPEN_LOOP_RAMP_PERIOD);
                                                       //.withTorqueOpenLoopRampPeriod(0);
        var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                           .withVoltageClosedLoopRampPeriod(SDC.CLOSED_LOOP_RAMP_PERIOD)
                                                           .withTorqueClosedLoopRampPeriod(0);
        var closedLoopGeneralConfig = new ClosedLoopGeneralConfigs().withContinuousWrap(true);
        var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                                  .withSensorToMechanismRatio(SDC.STEER_GEAR_RATIO)
                                                  .withRotorToSensorRatio(1.0);
        MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs().withNeutralMode(SDC.STEER_MOTOR_NEUTRAL_MODE)
                                                        .withInverted(SDC.STEER_MOTOR_INVERT)
                                                        .withPeakForwardDutyCycle(SDC.OUTPUT_ROTATE_LIMIT_FACTOR)
                                                        .withPeakReverseDutyCycle(-SDC.OUTPUT_ROTATE_LIMIT_FACTOR);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                                                        .withSupplyCurrentLimit(SDC.STEER_SUPPLY_CURRENT_LIMIT)
                                                        .withSupplyCurrentLimitEnable(SDC.STEER_ENABLE_SUPPLY_CURRENT_LIMIT)
                                                        .withStatorCurrentLimit(SDC.STEER_STATOR_CURRENT_LIMIT)
                                                        .withStatorCurrentLimitEnable(SDC.STEER_ENABLE_STATOR_CURRENT_LIMIT );
        Slot0Configs pid0Configs = new Slot0Configs().withKP(SDC.STEER_KP)
                                                     .withKI(SDC.STEER_KI)
                                                     .withKD(SDC.STEER_KD);
        var swerveDriveConfig = new TalonFXConfiguration().withFeedback(feedbackConfig)
                                                          .withMotorOutput(motorOutputConfig)
                                                          .withCurrentLimits(currentLimitConfig)
                                                          .withOpenLoopRamps(openLoopConfig)
                                                          .withClosedLoopRamps(closedLoopConfig)
                                                          .withClosedLoopGeneral(closedLoopGeneralConfig)
                                                          .withSlot0(pid0Configs);
        StatusCode status = m_steerMotor.getConfigurator().apply(swerveDriveConfig);

        if (! status.isOK() ) {
            SmartDashboard.putString("Failed to apply Steer configs in Mod "+m_modNum, " Error code: "+status.toString());
        }
    }

    private void configDriveMotor() {
        var openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                       .withVoltageOpenLoopRampPeriod(SDC.OPEN_LOOP_RAMP_PERIOD);
                                                        //.withTorqueOpenLoopRampPeriod(0);
        var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                           .withVoltageClosedLoopRampPeriod(SDC.CLOSED_LOOP_RAMP_PERIOD)
                                                           .withTorqueClosedLoopRampPeriod(0);
        var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                                  .withSensorToMechanismRatio(SDC.DRIVE_GEAR_RATIO)
                                                  .withRotorToSensorRatio(1.0);
        var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(SDC.DRIVE_MOTOR_NEUTRAL_MODE)
                                                        .withInverted(SDC.DRIVE_MOTOR_INVERT)
                                                        .withPeakForwardDutyCycle(SDC.OUTPUT_DRIVE_LIMIT_FACTOR)
                                                        .withPeakReverseDutyCycle(-SDC.OUTPUT_DRIVE_LIMIT_FACTOR);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                                                        .withSupplyCurrentLimit(SDC.DRIVE_SUPPLY_CURRENT_LIMIT)
                                                        .withSupplyCurrentLimitEnable(SDC.DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT)
                                                        .withStatorCurrentLimit(SDC.DRIVE_STATOR_CURRENT_LIMIT)
                                                        .withStatorCurrentLimitEnable(SDC.DRIVE_ENABLE_STATOR_CURRENT_LIMIT );
        Slot0Configs pid0Configs = new Slot0Configs().withKP(SDC.DRIVE_KP)
                                                     .withKI(SDC.DRIVE_KI)
                                                     .withKD(SDC.DRIVE_KD)
                                                     .withKS(SDC.DRIVE_KS)
                                                     .withKV(SDC.DRIVE_KV)
                                                     .withKA(SDC.DRIVE_KA)
                                                     .withKG(SDC.DRIVE_KG);
        var swerveDriveConfig = new TalonFXConfiguration().withFeedback(feedbackConfig)
                                                          .withMotorOutput(motorOutputConfig)
                                                          .withCurrentLimits(currentLimitConfig)
                                                          .withOpenLoopRamps(openLoopConfig)
                                                          .withClosedLoopRamps(closedLoopConfig)
                                                          .withSlot0(pid0Configs);
        StatusCode status = m_driveMotor.getConfigurator().apply(swerveDriveConfig);

        if (! status.isOK() ) {
            SmartDashboard.putString("Failed to apply Drive configs in Mod "+m_modNum, " Error code: "+status.toString());
        }
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getVelocityMPS(),
            getAngle2d()
        ); 
    }

    public double getPositionM() {
        return m_driveMotor.getPosition().getValueAsDouble() * SDC.TALONFX_ROT_TO_M_FACTOR; 
    }

    public double getVelocityMPS() {
        return m_driveMotor.getVelocity().getValueAsDouble() * SDC.TALONFX_RPS_TO_MPS_FACTOR;
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getPositionM(), getAngle2d());
    }

    public void setupModulePublishing() {
        ShuffleboardLayout sBE_Layout = m_moduleConstants.SBE_LAYOUT;
        // See comment above about not needing to retain these Entry keys
        /* IdsEntry (D R E)= */ sBE_Layout.add("Ids", 
                                               F.df1.format(m_moduleConstants.DRIVE_MOTOR_ID)+
                                               "  "+F.df1.format(m_moduleConstants.STEER_MOTOR_ID)+
                                               "  "+F.df1.format(m_moduleConstants.ENCODER_ID));
        /* absOffsetEntry = */  sBE_Layout.add("Offset", F.df1.format(m_moduleConstants.ABS_ANG_OFFSET2D.getDegrees()));
        absCANcoderDegEntry =   sBE_Layout.add("CCdeg", "0").getEntry();
        steerEncoderDegEntry =  sBE_Layout.add("MEdeg", "0").getEntry();
        steerSetpointDegEntry = sBE_Layout.add("SPdeg", "0").getEntry();
        steerPIDOutputEntry =   sBE_Layout.add("PID_O", "0").getEntry();
        wheelCurrSpeedEntry =   sBE_Layout.add("Wspd", "0").getEntry();
        wheelCurrPosEntry =     sBE_Layout.add("Wpos", "0").getEntry();
        wheelAmpsEntry =        sBE_Layout.add("DrAmps", "0").getEntry();
        wheelTempEntry =        sBE_Layout.add("DrTemp", "0").getEntry();
        steerAmpsEntry =        sBE_Layout.add("R_Amps", "0").getEntry();
        steerTempEntry =        sBE_Layout.add("R_Temp", "0").getEntry();
    }

    public void publishModuleData() {
        // CANcoder direction
        absCANcoderDegEntry.setString(F.df1.format(getCANcoderDeg()));
        // Current wheel direction
        steerEncoderDegEntry.setString(F.df1.format(getFalconPosDeg())); 
        // Wheel direction (steer) setpoint
        steerSetpointDegEntry.setString(F.df1.format(normalizeAngle0To360(m_lastAngle)));
        //steerSetpointDegEntry.setString(F.df1.format(m_lastAngle));
        // SteerMotor PID applied ouutput    
        steerPIDOutputEntry.setString(F.df2.format(m_steerMotor.getMotorOutputStatus().getValueAsDouble()));
        // Wheel Speed
        wheelCurrSpeedEntry.setString(F.df1.format(getVelocityMPS()));
        // Wheel position, meters
        wheelCurrPosEntry.setString(F.df2.format(getPositionM()));
        // Wheel Amps
        wheelAmpsEntry.setString(F.df1.format(m_driveMotor.getSupplyCurrent().getValueAsDouble()));
        // Wheel Temp
        wheelTempEntry.setString(F.df1.format(m_driveMotor.getDeviceTemp().getValueAsDouble()));
        // Steering Amps
        steerAmpsEntry.setString(F.df1.format(m_steerMotor.getSupplyCurrent().getValueAsDouble()));
        // Steering Temp
        steerTempEntry.setString(F.df1.format(m_steerMotor.getDeviceTemp().getValueAsDouble()));
    }

    public void stop() {
        m_driveOpenLoop.Output = 0.0;
        m_driveMotor.setControl(m_driveOpenLoop);
        m_steerMotor.stopMotor();
    }
}