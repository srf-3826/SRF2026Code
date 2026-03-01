package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import frc.robot.Constants;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
private TalonFX m_arm ; // 2.5:1 on a 27:1, kraken
private TalonFXS m_rollers; // 9:1, minion
private TalonFXS m_hopper; // gear ration 3:1, neo
private CANcoder m_encoder;
private CANBus m_intakeBus;
private VelocityVoltage m_control_roller;
private PositionVoltage m_control_pivot;
        
    IntakeSubsystem(CANBus intakeBus) {
        m_intakeBus = intakeBus;
        TalonFX m_arm = new TalonFX(Constants.ISC.INTAKE_PIVOT_MOTOR_ID, intakeBus);
        TalonFXS m_rollers = new TalonFXS(Constants.ISC.INTAKE_ROLLER_MOTOR_ID, intakeBus);
        TalonFXS m_hopper = new TalonFXS(Constants.ISC.HOPPER_FLOOR_MOTOR_ID, intakeBus);
        CANcoder m_encoder = new CANcoder(Constants.ISC.ARM_ENCODER_ID, intakeBus);
        configPivotMotor();
        configHopperMotor();
        configRollerMotor();
        configArmEncoder();
    }

    public class IntakeChassis {
        public static void setup() {

        }
        public static void execute() {

        }
        public static void gotoRetract() {

        }
        public static void gotoFloorPickup() {

        }
        public static void gotoIdle() {

        }
    }
    public class IntakeRollers {
        public static void setup() {
            
        }
        public static void execute() {

        }
        public static void expelFuel() {

        }
        public static void halt() {

        }
        public static void intakeFuel() {

        }
    }
    public class HopperRollers {
        public static void setup() {
            
        }
        public static void execute() {

        }
        public static void expelFuel() {

        }
        public static void halt() {

        }
        public static void alignFuelToShoot() {

        }
    }

    public void periodic(){

    }

    private void configPivotMotor() {
        var openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                       .withVoltageOpenLoopRampPeriod(Constants.ISC.PIVOT_OPEN_LOOP_RAMP_PERIOD);
                                                       //.withTorqueOpenLoopRampPeriod(0);
        var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                           .withVoltageClosedLoopRampPeriod(Constants.ISC.PIVOT_CLOSED_LOOP_RAMP_PERIOD)
                                                           .withTorqueClosedLoopRampPeriod(0);
        var feedbackConfig = new FeedbackConfigs().withFeedbackRemoteSensorID(Constants.ISC.ARM_ENCODER_ID)
                                                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                                                  .withSensorToMechanismRatio(1)
                                                  .withRotorToSensorRatio(Constants.ISC.PIVOT_GEAR_RATIO);
        var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(Constants.ISC.PIVOT_MOTOR_NEUTRAL_MODE)
                                                        .withInverted(Constants.ISC.PIVOT_MOTOR_INVERT)
                                                        .withPeakForwardDutyCycle(Constants.ISC.PIVOT_OUTPUT_MOTOR_LIMIT_FACTOR)
                                                        .withPeakReverseDutyCycle(-Constants.ISC.PIVOT_OUTPUT_MOTOR_LIMIT_FACTOR);
                                                        //.withDutyCycleNeutralDeadband(.001);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                                                        .withSupplyCurrentLimit(Constants.ISC.PIVOT_MOTOR_SUPPLY_CURRENT_LIMIT)
                                                        .withSupplyCurrentLimitEnable(Constants.ISC.PIVOT_ENABLE_SUPPLY_CURRENT_LIMIT)
                                                        .withStatorCurrentLimit(Constants.ISC.PIVOT_STATOR_CURRENT_LIMIT)
                                                        .withStatorCurrentLimitEnable(Constants.ISC.PIVOT_ENABLE_STATOR_CURRENT_LIMIT);
        Slot0Configs pid0Configs = new Slot0Configs().withKP(Constants.ISC.PIVOT_MOTOR_KP)
                                                     .withKI(Constants.ISC.PIVOT_MOTOR_KI)
                                                     .withKD(Constants.ISC.PIVOT_MOTOR_KD)
                                                     .withKS(Constants.ISC.PIVOT_MOTOR_KS)
                                                     .withKV(Constants.ISC.PIVOT_MOTOR_KV)
                                                     .withKA(Constants.ISC.PIVOT_MOTOR_KA)
                                                     .withGravityType(GravityTypeValue.Arm_Cosine)
                                                     .withKG(Constants.ISC.PIVOT_MOTOR_KG);
        var IntakeArmConfig = new TalonFXConfiguration().withFeedback(feedbackConfig)
                                                        .withMotorOutput(motorOutputConfig)
                                                        .withCurrentLimits(currentLimitConfig)
                                                        .withOpenLoopRamps(openLoopConfig)
                                                        .withClosedLoopRamps(closedLoopConfig)
                                                        .withSlot0(pid0Configs);

        StatusCode status = m_arm.getConfigurator().apply(IntakeArmConfig);

        if (! status.isOK()) System.out.println("IntakeArm motor config: "
                                                +status.getDescription());
    }

     private void configRollerMotor() {
        OpenLoopRampsConfigs openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                       .withVoltageOpenLoopRampPeriod(Constants.ISC.ROLLER_OPEN_LOOP_RAMP_PERIOD);
                                                       //.withTorqueOpenLoopRampPeriod(0);
        ClosedLoopRampsConfigs closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                           .withVoltageClosedLoopRampPeriod(Constants.ISC.ROLLER_CLOSED_LOOP_RAMP_PERIOD)
                                                           .withTorqueClosedLoopRampPeriod(0);
        var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                                  .withSensorToMechanismRatio(Constants.ISC.ROLLER_GEAR_RATIO)
                                                  .withRotorToSensorRatio(1.0);
        MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs().withNeutralMode(Constants.ISC.ROLLER_MOTOR_NEUTRAL_MODE)
                                                        .withInverted(Constants.ISC.ROLLER_MOTOR_INVERT)
                                                        .withPeakForwardDutyCycle(Constants.ISC.ROLLER_OUTPUT_MOTOR_LIMIT_FACTOR)
                                                        .withPeakReverseDutyCycle(-Constants.ISC.ROLLER_OUTPUT_MOTOR_LIMIT_FACTOR);
                                                        //.withDutyCycleNeutralDeadband(.001);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                                                        .withSupplyCurrentLimit(Constants.ISC.ROLLER_MOTOR_SUPPLY_CURRENT_LIMIT)
                                                        .withSupplyCurrentLimitEnable(Constants.ISC.ROLLER_ENABLE_SUPPLY_CURRENT_LIMIT)
                                                        .withStatorCurrentLimit(Constants.ISC.ROLLER_STATOR_CURRENT_LIMIT)
                                                        .withStatorCurrentLimitEnable(Constants.ISC.ROLLER_ENABLE_STATOR_CURRENT_LIMIT);
        Slot0Configs pid0Configs = new Slot0Configs().withKP(Constants.ISC.ROLLER_MOTOR_KP)
                                                     .withKI(Constants.ISC.ROLLER_MOTOR_KI)
                                                     .withKD(Constants.ISC.ROLLER_MOTOR_KD)
                                                     .withKS(Constants.ISC.ROLLER_MOTOR_KS)
                                                     .withKV(Constants.ISC.ROLLER_MOTOR_KV)
                                                     .withKA(Constants.ISC.ROLLER_MOTOR_KA);
        var IntakeRollersConfig = new TalonFXSConfiguration().withMotorOutput(motorOutputConfig)
                                                          .withCurrentLimits(currentLimitConfig)
                                                          .withOpenLoopRamps(openLoopConfig)
                                                          .withClosedLoopRamps(closedLoopConfig)
                                                          .withSlot0(pid0Configs);

        StatusCode status = m_rollers.getConfigurator().apply(IntakeRollersConfig);

        if (! status.isOK()) System.out.println("IntakeRollers motor config: "
                                                +status.getDescription());
    }
                                                
    private void configHopperMotor() {
        OpenLoopRampsConfigs openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                       .withVoltageOpenLoopRampPeriod(Constants.ISC.HOPPER_OPEN_LOOP_RAMP_PERIOD);
                                                       //.withTorqueOpenLoopRampPeriod(0);
        ClosedLoopRampsConfigs closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                           .withVoltageClosedLoopRampPeriod(Constants.ISC.HOPPER_CLOSED_LOOP_RAMP_PERIOD)
                                                           .withTorqueClosedLoopRampPeriod(0);
        FeedbackConfigs feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                                  .withSensorToMechanismRatio(Constants.ISC.HOPPER_GEAR_RATIO)
                                                  .withRotorToSensorRatio(1.0);
        MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs().withNeutralMode(Constants.ISC.HOPPER_MOTOR_NEUTRAL_MODE)
                                                        .withInverted(Constants.ISC.HOPPER_MOTOR_INVERT)
                                                        .withPeakForwardDutyCycle(Constants.ISC.HOPPER_OUTPUT_MOTOR_LIMIT_FACTOR)
                                                        .withPeakReverseDutyCycle(-Constants.ISC.HOPPER_OUTPUT_MOTOR_LIMIT_FACTOR);
                                                        //.withDutyCycleNeutralDeadband(.001);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                                                        .withSupplyCurrentLimit(Constants.ISC.HOPPER_MOTOR_SUPPLY_CURRENT_LIMIT)
                                                        .withSupplyCurrentLimitEnable(Constants.ISC.HOPPER_ENABLE_SUPPLY_CURRENT_LIMIT)
                                                        .withStatorCurrentLimit(Constants.ISC.HOPPER_STATOR_CURRENT_LIMIT)
                                                        .withStatorCurrentLimitEnable(Constants.ISC.HOPPER_ENABLE_STATOR_CURRENT_LIMIT);
        Slot0Configs pid0Configs = new Slot0Configs().withKP(Constants.ISC.HOPPER_MOTOR_KP)
                                                     .withKI(Constants.ISC.HOPPER_MOTOR_KI)
                                                     .withKD(Constants.ISC.HOPPER_MOTOR_KD)
                                                     .withKS(Constants.ISC.HOPPER_MOTOR_KS)
                                                     .withKV(Constants.ISC.HOPPER_MOTOR_KV)
                                                     .withKA(Constants.ISC.HOPPER_MOTOR_KA);
        var HopperFloorConfig = new TalonFXSConfiguration().withMotorOutput(motorOutputConfig)
                                                           .withCurrentLimits(currentLimitConfig)
                                                           .withOpenLoopRamps(openLoopConfig)
                                                           .withClosedLoopRamps(closedLoopConfig)
                                                           .withSlot0(pid0Configs);

        StatusCode status = m_hopper.getConfigurator().apply(HopperFloorConfig);

        if (! status.isOK()) System.out.println("HopperFloor motor config: "
                                                +status.getDescription());
    }

    private void configArmEncoder() {
        MagnetSensorConfigs magnetSensorConfig = new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(Constants.ISC.ARM_ABSOLUTE_SENSOR_DISCONTINUITY_POINT)
                                                                          .withMagnetOffset(Constants.ISC.ARM_ENCODER_MAGNET_OFFSET);
                                                       //.withTorqueOpenLoopRampPeriod(0);
       
        var ArmEncoderConfig = new CANcoderConfiguration().withMagnetSensor(magnetSensorConfig);

        StatusCode status = m_encoder.getConfigurator().apply(ArmEncoderConfig);

        if (! status.isOK()) System.out.println("HopperFloor motor config: "
                                                +status.getDescription());
    }
}

