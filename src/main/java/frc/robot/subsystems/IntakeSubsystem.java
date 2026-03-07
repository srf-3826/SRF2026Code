package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import frc.robot.Constants.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
private TalonFX m_arm ; // 2.5:1 on a 27:1, kraken
private TalonFXS m_rollers; // 9:1, minion
private TalonFXS m_hopper; // gear ration 3:1, neo
private CANcoder m_encoder;
private CANBus m_intakeBus;
private VelocityVoltage m_control_roller;
private PositionVoltage m_control_pivot;

double arm_target_angle = 0;
        
    public IntakeSubsystem(CANBus intakeBus) {
        m_intakeBus = intakeBus;
        m_arm = new TalonFX(ISC.INTAKE_PIVOT_MOTOR_ID, intakeBus);
        m_rollers = new TalonFXS(ISC.INTAKE_ROLLER_MOTOR_ID, intakeBus);
        m_hopper = new TalonFXS(ISC.HOPPER_FLOOR_MOTOR_ID, intakeBus);
        m_encoder = new CANcoder(ISC.ARM_ENCODER_ID, intakeBus);
        configPivotMotor();
        configHopperMotor();
        configRollerMotor();
        configArmEncoder();
    }

    public void arm_gotoRetract() {
        MotionMagicVoltage request = new MotionMagicVoltage(0);
        m_arm.setControl(request.withPosition(ISC.PIVOT_MOTOR_RETRACT_ANGLE));
    }
    public void arm_gotoFloorPickup() {
        MotionMagicVoltage request = new MotionMagicVoltage(0);
        m_arm.setControl(request.withPosition(ISC.PIVOT_MOTOR_FLOOR_ANGLE));
    }
    public void arm_gotoHold() {
        MotionMagicVoltage request = new MotionMagicVoltage(0);
        m_arm.setControl(request.withPosition(ISC.PIVOT_MOTOR_HOLD_ANGLE));
    }
    
   
    public void intake_ExpelFuel() {
        MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
        m_rollers.setControl(request.withVelocity(-ISC.ROLLER_SPEED));
    }
    public void intake_Halt() {
        MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
        m_rollers.setControl(request.withVelocity(0));
    }
    public void intake_IntakeFuel() {
        MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
        m_rollers.setControl(request.withVelocity(ISC.ROLLER_SPEED));
    }
    
    public void hopper_ExpelFuel() {
    MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
        m_rollers.setControl(request.withVelocity(-ISC.HOPPER_SPEED));
    }
    public void hopper_Halt() {
        MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
        m_rollers.setControl(request.withVelocity(0));
    }
    public void hopper_AlignFuelToShoot() {
        MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
        m_rollers.setControl(request.withVelocity(ISC.HOPPER_SPEED));
    }

    private void configPivotMotor() {
        var openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                       .withVoltageOpenLoopRampPeriod(ISC.PIVOT_OPEN_LOOP_RAMP_PERIOD);
                                                       //.withTorqueOpenLoopRampPeriod(0);
        var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                           .withVoltageClosedLoopRampPeriod(ISC.PIVOT_CLOSED_LOOP_RAMP_PERIOD)
                                                           .withTorqueClosedLoopRampPeriod(0);
        var feedbackConfig = new FeedbackConfigs().withFeedbackRemoteSensorID(ISC.ARM_ENCODER_ID)
                                                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                                                  .withSensorToMechanismRatio(1)
                                                  .withRotorToSensorRatio(ISC.PIVOT_GEAR_RATIO);
        var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(ISC.PIVOT_MOTOR_NEUTRAL_MODE)
                                                        .withInverted(ISC.PIVOT_MOTOR_INVERT)
                                                        .withPeakForwardDutyCycle(ISC.PIVOT_OUTPUT_MOTOR_LIMIT_FACTOR)
                                                        .withPeakReverseDutyCycle(-ISC.PIVOT_OUTPUT_MOTOR_LIMIT_FACTOR);
                                                        //.withDutyCycleNeutralDeadband(.001);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                                                        .withSupplyCurrentLimit(ISC.PIVOT_MOTOR_SUPPLY_CURRENT_LIMIT)
                                                        .withSupplyCurrentLimitEnable(ISC.PIVOT_ENABLE_SUPPLY_CURRENT_LIMIT)
                                                        .withStatorCurrentLimit(ISC.PIVOT_STATOR_CURRENT_LIMIT)
                                                        .withStatorCurrentLimitEnable(ISC.PIVOT_ENABLE_STATOR_CURRENT_LIMIT);
        Slot0Configs pid0Configs = new Slot0Configs().withKP(ISC.PIVOT_MOTOR_KP)
                                                     .withKI(ISC.PIVOT_MOTOR_KI)
                                                     .withKD(ISC.PIVOT_MOTOR_KD)
                                                     .withKS(ISC.PIVOT_MOTOR_KS)
                                                     .withKV(ISC.PIVOT_MOTOR_KV)
                                                     .withKA(ISC.PIVOT_MOTOR_KA)
                                                     .withGravityType(GravityTypeValue.Arm_Cosine)
                                                     .withKG(ISC.PIVOT_MOTOR_KG);
        var IntakeArmConfig = new TalonFXConfiguration().withFeedback(feedbackConfig)
                                                        .withMotorOutput(motorOutputConfig)
                                                        .withCurrentLimits(currentLimitConfig)
                                                        .withOpenLoopRamps(openLoopConfig)
                                                        .withClosedLoopRamps(closedLoopConfig)
                                                        .withSlot0(pid0Configs);
        IntakeArmConfig.MotionMagic.MotionMagicCruiseVelocity = ISC.PIVOT_MOTOR_MAX_VEL;
        IntakeArmConfig.MotionMagic.MotionMagicAcceleration = ISC.PIVOT_MOTOR_ACCEL;
        IntakeArmConfig.MotionMagic.MotionMagicJerk = ISC.PIVOT_MOTOR_JERK;

        StatusCode status = m_arm.getConfigurator().apply(IntakeArmConfig);

        if (! status.isOK()) System.out.println("IntakeArm motor config: "
                                                +status.getDescription());
    }

     private void configRollerMotor() {
        OpenLoopRampsConfigs openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                       .withVoltageOpenLoopRampPeriod(ISC.ROLLER_OPEN_LOOP_RAMP_PERIOD);
                                                       //.withTorqueOpenLoopRampPeriod(0);
        ClosedLoopRampsConfigs closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                           .withVoltageClosedLoopRampPeriod(ISC.ROLLER_CLOSED_LOOP_RAMP_PERIOD)
                                                           .withTorqueClosedLoopRampPeriod(0);
        var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                                  .withSensorToMechanismRatio(ISC.ROLLER_GEAR_RATIO)
                                                  .withRotorToSensorRatio(1.0);
        MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs().withNeutralMode(ISC.ROLLER_MOTOR_NEUTRAL_MODE)
                                                        .withInverted(ISC.ROLLER_MOTOR_INVERT)
                                                        .withPeakForwardDutyCycle(ISC.ROLLER_OUTPUT_MOTOR_LIMIT_FACTOR)
                                                        .withPeakReverseDutyCycle(-ISC.ROLLER_OUTPUT_MOTOR_LIMIT_FACTOR);
                                                        //.withDutyCycleNeutralDeadband(.001);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                                                        .withSupplyCurrentLimit(ISC.ROLLER_MOTOR_SUPPLY_CURRENT_LIMIT)
                                                        .withSupplyCurrentLimitEnable(ISC.ROLLER_ENABLE_SUPPLY_CURRENT_LIMIT)
                                                        .withStatorCurrentLimit(ISC.ROLLER_STATOR_CURRENT_LIMIT)
                                                        .withStatorCurrentLimitEnable(ISC.ROLLER_ENABLE_STATOR_CURRENT_LIMIT);
        Slot0Configs pid0Configs = new Slot0Configs().withKP(ISC.ROLLER_MOTOR_KP)
                                                     .withKI(ISC.ROLLER_MOTOR_KI)
                                                     .withKD(ISC.ROLLER_MOTOR_KD)
                                                     .withKS(ISC.ROLLER_MOTOR_KS)
                                                     .withKV(ISC.ROLLER_MOTOR_KV)
                                                     .withKA(ISC.ROLLER_MOTOR_KA);
        var IntakeRollersConfig = new TalonFXSConfiguration().withMotorOutput(motorOutputConfig)
                                                          .withCurrentLimits(currentLimitConfig)
                                                          .withOpenLoopRamps(openLoopConfig)
                                                          .withClosedLoopRamps(closedLoopConfig)
                                                          .withSlot0(pid0Configs);
        IntakeRollersConfig.MotionMagic.MotionMagicCruiseVelocity = ISC.ROLLER_MOTOR_MAX_VEL;
        IntakeRollersConfig.MotionMagic.MotionMagicAcceleration = ISC.ROLLER_MOTOR_ACCEL;
        IntakeRollersConfig.MotionMagic.MotionMagicJerk = ISC.ROLLER_MOTOR_JERK;

        StatusCode status = m_rollers.getConfigurator().apply(IntakeRollersConfig);

        if (! status.isOK()) System.out.println("IntakeRollers motor config: "
                                                +status.getDescription());
    }
                                                
    private void configHopperMotor() {
        OpenLoopRampsConfigs openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                       .withVoltageOpenLoopRampPeriod(ISC.HOPPER_OPEN_LOOP_RAMP_PERIOD);
                                                       //.withTorqueOpenLoopRampPeriod(0);
        ClosedLoopRampsConfigs closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                           .withVoltageClosedLoopRampPeriod(ISC.HOPPER_CLOSED_LOOP_RAMP_PERIOD)
                                                           .withTorqueClosedLoopRampPeriod(0);
        FeedbackConfigs feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                                  .withSensorToMechanismRatio(ISC.HOPPER_GEAR_RATIO)
                                                  .withRotorToSensorRatio(1.0);
        MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs().withNeutralMode(ISC.HOPPER_MOTOR_NEUTRAL_MODE)
                                                        .withInverted(ISC.HOPPER_MOTOR_INVERT)
                                                        .withPeakForwardDutyCycle(ISC.HOPPER_OUTPUT_MOTOR_LIMIT_FACTOR)
                                                        .withPeakReverseDutyCycle(-ISC.HOPPER_OUTPUT_MOTOR_LIMIT_FACTOR);
                                                        //.withDutyCycleNeutralDeadband(.001);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                                                        .withSupplyCurrentLimit(ISC.HOPPER_MOTOR_SUPPLY_CURRENT_LIMIT)
                                                        .withSupplyCurrentLimitEnable(ISC.HOPPER_ENABLE_SUPPLY_CURRENT_LIMIT)
                                                        .withStatorCurrentLimit(ISC.HOPPER_STATOR_CURRENT_LIMIT)
                                                        .withStatorCurrentLimitEnable(ISC.HOPPER_ENABLE_STATOR_CURRENT_LIMIT);
        Slot0Configs pid0Configs = new Slot0Configs().withKP(ISC.HOPPER_MOTOR_KP)
                                                     .withKI(ISC.HOPPER_MOTOR_KI)
                                                     .withKD(ISC.HOPPER_MOTOR_KD)
                                                     .withKS(ISC.HOPPER_MOTOR_KS)
                                                     .withKV(ISC.HOPPER_MOTOR_KV)
                                                     .withKA(ISC.HOPPER_MOTOR_KA);
        var HopperFloorConfig = new TalonFXSConfiguration().withMotorOutput(motorOutputConfig)
                                                           .withCurrentLimits(currentLimitConfig)
                                                           .withOpenLoopRamps(openLoopConfig)
                                                           .withClosedLoopRamps(closedLoopConfig)
                                                           .withSlot0(pid0Configs);
        HopperFloorConfig.MotionMagic.MotionMagicCruiseVelocity = ISC.HOPPER_MOTOR_MAX_VEL;
        HopperFloorConfig.MotionMagic.MotionMagicAcceleration = ISC.HOPPER_MOTOR_ACCEL;
        HopperFloorConfig.MotionMagic.MotionMagicJerk = ISC.HOPPER_MOTOR_JERK;

        StatusCode status = m_hopper.getConfigurator().apply(HopperFloorConfig);

        if (! status.isOK()) System.out.println("HopperFloor motor config: "
                                                +status.getDescription());
    }

    private void configArmEncoder() {
         MagnetSensorConfigs magnetSensorConfig = new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(ISC.ARM_ABSOLUTE_SENSOR_DISCONTINUITY_POINT)
                                                                           .withMagnetOffset(ISC.ARM_ENCODER_MAGNET_OFFSET)
                                                                           .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
                                                       
       
         var ArmEncoderConfig = new CANcoderConfiguration().withMagnetSensor(magnetSensorConfig);

         StatusCode status = m_encoder.getConfigurator().apply(ArmEncoderConfig);

         if (! status.isOK()) System.out.println("IntakeArm Encoder config: "
                                                +status.getDescription());
    }
}

