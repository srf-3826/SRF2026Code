package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ISC;
import frc.robot.Constants.SSC;

public class ShooterSubsystem  extends SubsystemBase {
    private double targetDistance;
    private double rpm;
    private TalonFX m_leftFlywheel;
    private TalonFX m_rightFlywheel;
    private TalonFXS m_leftFeedWheel;
    private TalonFXS m_rightFeedWheel;
    private CANBus m_shooterBus;

    public ShooterSubsystem(CANBus shooterBus) {
        m_shooterBus = shooterBus;
        m_leftFlywheel = new TalonFX(SSC.LEFT_SHOOTER_MOTOR_ID, shooterBus);
        m_rightFlywheel = new TalonFX(SSC.RIGHT_SHOOTER_MOTOR_ID, shooterBus);
        m_leftFeedWheel = new TalonFXS(SSC.LEFT_FEED_MOTOR_ID, shooterBus);
        m_rightFeedWheel = new TalonFXS(SSC.RIGHT_FEED_MOTOR_ID, shooterBus);
        configFlywheels();
        configFeedMotors();
    }
    
    
    /**
     * 
     * @param distance Distance in meters, for scoring in the HUB
     */
    
     private void configFlywheels() {
        var openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                       .withVoltageOpenLoopRampPeriod(SSC.FLY_OPEN_LOOP_RAMP_PERIOD);
                                                       //.withTorqueOpenLoopRampPeriod(0);
        var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                           .withVoltageClosedLoopRampPeriod(SSC.FLY_CLOSED_LOOP_RAMP_PERIOD)
                                                           .withTorqueClosedLoopRampPeriod(0);
        var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                                  .withSensorToMechanismRatio(1)
                                                  .withRotorToSensorRatio(SSC.FLY_GEAR_RATIO);
        var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(SSC.FLY_MOTOR_NEUTRAL_MODE)
                                                        .withInverted(SSC.FLY_LEFT_MOTOR_INVERT)
                                                        .withPeakForwardDutyCycle(SSC.FLY_OUTPUT_MOTOR_LIMIT_FACTOR)
                                                        .withPeakReverseDutyCycle(-SSC.FLY_OUTPUT_MOTOR_LIMIT_FACTOR);
                                                        //.withDutyCycleNeutralDeadband(.001);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                                                        .withSupplyCurrentLimit(SSC.FLY_MOTOR_SUPPLY_CURRENT_LIMIT)
                                                        .withSupplyCurrentLimitEnable(SSC.FLY_ENABLE_SUPPLY_CURRENT_LIMIT)
                                                        .withStatorCurrentLimit(SSC.FLY_STATOR_CURRENT_LIMIT)
                                                        .withStatorCurrentLimitEnable(SSC.FLY_ENABLE_STATOR_CURRENT_LIMIT);
        Slot0Configs pid0Configs = new Slot0Configs().withKP(SSC.FLY_MOTOR_KP)
                                                     .withKI(SSC.FLY_MOTOR_KI)
                                                     .withKD(SSC.FLY_MOTOR_KD)
                                                     .withKS(SSC.FLY_MOTOR_KS)
                                                     .withKV(SSC.FLY_MOTOR_KV)
                                                     .withKA(SSC.FLY_MOTOR_KA)
                                                     .withKG(SSC.FLY_MOTOR_KG);
        var leftFlyConfig = new TalonFXConfiguration().withMotorOutput(motorOutputConfig)
                                                      .withCurrentLimits(currentLimitConfig)
                                                      .withOpenLoopRamps(openLoopConfig)
                                                      .withClosedLoopRamps(closedLoopConfig)
                                                      .withSlot0(pid0Configs);
        var rightFlyConfig = new TalonFXConfiguration().withMotorOutput(motorOutputConfig.withInverted(SSC.FLY_RIGHT_MOTOR_INVERT))
                                                       .withCurrentLimits(currentLimitConfig)
                                                       .withOpenLoopRamps(openLoopConfig)
                                                       .withClosedLoopRamps(closedLoopConfig)
                                                       .withSlot0(pid0Configs);
        
        StatusCode status = m_leftFlywheel.getConfigurator().apply(leftFlyConfig);

        if (! status.isOK()) System.out.println("Left Flywheel motor config: "
                                                +status.getDescription());
        StatusCode statusRight = m_rightFlywheel.getConfigurator().apply(rightFlyConfig);

        if (! statusRight.isOK()) System.out.println("Right Flyhweel motor config "
                                                     +statusRight.getDescription());
    }

private void configFeedMotors() {
        OpenLoopRampsConfigs openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                       .withVoltageOpenLoopRampPeriod(SSC.FEED_OPEN_LOOP_RAMP_PERIOD);
                                                       //.withTorqueOpenLoopRampPeriod(0);
        ClosedLoopRampsConfigs closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                           .withVoltageClosedLoopRampPeriod(SSC.FEED_CLOSED_LOOP_RAMP_PERIOD)
                                                           .withTorqueClosedLoopRampPeriod(0);
        FeedbackConfigs feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                                  .withSensorToMechanismRatio(SSC.FEED_GEAR_RATIO)
                                                  .withRotorToSensorRatio(1.0);
        MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs().withNeutralMode(SSC.FEED_MOTOR_NEUTRAL_MODE)
                                                        .withInverted(SSC.LEFT_FEED_MOTOR_INVERT)
                                                        .withPeakForwardDutyCycle(SSC.FEED_OUTPUT_MOTOR_LIMIT_FACTOR)
                                                        .withPeakReverseDutyCycle(-SSC.FEED_OUTPUT_MOTOR_LIMIT_FACTOR);
                                                        //.withDutyCycleNeutralDeadband(.001);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(SSC.FEED_MOTOR_SUPPLY_CURRENT_LIMIT)
                                                                            .withSupplyCurrentLimitEnable(SSC.FEED_ENABLE_SUPPLY_CURRENT_LIMIT)
                                                                            .withStatorCurrentLimit(SSC.FEED_STATOR_CURRENT_LIMIT)
                                                                            .withStatorCurrentLimitEnable(SSC.FEED_ENABLE_STATOR_CURRENT_LIMIT);
        Slot0Configs pid0Configs = new Slot0Configs().withKP(SSC.FEED_MOTOR_KP)
                                                     .withKI(SSC.FEED_MOTOR_KI)
                                                     .withKD(SSC.FEED_MOTOR_KD)
                                                     .withKS(SSC.FEED_MOTOR_KS)
                                                     .withKV(SSC.FEED_MOTOR_KV)
                                                     .withKA(SSC.FEED_MOTOR_KA);
        CommutationConfigs commutationConfigs = new CommutationConfigs().withAdvancedHallSupport(SSC.FEED_ADVANCED_HALL_SUPPORT_VALUE)
                                                                        .withBrushedMotorWiring(null)
                                                                        .withMotorArrangement(SSC.FEED_MOTOR_ARRANGEMENT_VALUE);
        var leftFeedConfig = new TalonFXSConfiguration().withMotorOutput(motorOutputConfig)
                                                          .withCurrentLimits(currentLimitConfig)
                                                          .withOpenLoopRamps(openLoopConfig)
                                                          .withClosedLoopRamps(closedLoopConfig)
                                                          .withSlot0(pid0Configs)
                                                          .withCommutation(commutationConfigs);
        var rightFeedConfig = new TalonFXSConfiguration().withMotorOutput(motorOutputConfig.withInverted(SSC.FLY_RIGHT_MOTOR_INVERT))
                                                         .withCurrentLimits(currentLimitConfig)
                                                         .withOpenLoopRamps(openLoopConfig)
                                                         .withClosedLoopRamps(closedLoopConfig)
                                                         .withSlot0(pid0Configs)
                                                         .withCommutation(commutationConfigs);

        StatusCode status = m_leftFeedWheel.getConfigurator().apply(leftFeedConfig);
        if (! status.isOK()) System.out.println("Left Feed motor config: "
                                                +status.getDescription());
        
        StatusCode statusRight = m_rightFeedWheel.getConfigurator().apply(rightFeedConfig);
        if (! statusRight.isOK()) System.out.println("Right Feed Motor config "
                                                    +statusRight.getDescription());

    }
    
    public void prepareDistantShot(double distance)
    {
        targetDistance = distance;
        double rps = distanceToRPS(targetDistance);
        setFlywheelVelocity(rps, 0);
        setFlywheelVelocity(rps, 1);
    }
    /**
     * 
     * @param velocity 
     * @param selector 0 for left, 1 for right
     */
    public void setFlywheelVelocity(double velocity, int selector) {
        final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
        m_request.withVelocity(velocity);
        if (selector == 0) {
            m_leftFlywheel.setControl(m_request);
        } else {
            m_rightFlywheel.setControl(m_request);
        }


    }
    private double distanceToRPS(double distance)
    {
        return distance; // TODO: make the equation, either through a lookup table, pure math, or interpolation.
    }
}
