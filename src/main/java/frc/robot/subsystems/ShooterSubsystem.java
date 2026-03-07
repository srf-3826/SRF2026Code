package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SSC;

public class ShooterSubsystem  extends SubsystemBase {
    private double targetDistance;
    private double rpm;
    private TalonFX m_leftFlywheel;
    private TalonFX m_rightFlywheel;

    ShooterSubsystem() {
        setupFlywheels();
    }
    private void setupFlywheels() {
        var openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                       .withVoltageOpenLoopRampPeriod(SSC.FLY_OPEN_LOOP_RAMP_PERIOD);
                                                       //.withTorqueOpenLoopRampPeriod(0);
        var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                           .withVoltageClosedLoopRampPeriod(SSC.FLY_CLOSED_LOOP_RAMP_PERIOD)
                                                           .withTorqueClosedLoopRampPeriod(0);
        var feedbackConfig = new FeedbackConfigs().withFeedbackRemoteSensorID(SSC.LEFT_ENCODER_ID)
                                                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
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
        var leftFlyConfig = new TalonFXConfiguration().withFeedback(feedbackConfig.withFeedbackRemoteSensorID(SSC.RIGHT_ENCODER_ID))
                                                        .withMotorOutput(motorOutputConfig)
                                                        .withCurrentLimits(currentLimitConfig)
                                                        .withOpenLoopRamps(openLoopConfig)
                                                        .withClosedLoopRamps(closedLoopConfig)
                                                        .withSlot0(pid0Configs);
        var rightFlyConfig = new TalonFXConfiguration().withFeedback(feedbackConfig)
                                                        .withMotorOutput(motorOutputConfig.withInverted(SSC.FLY_RIGHT_MOTOR_INVERT))
                                                        .withCurrentLimits(currentLimitConfig)
                                                        .withOpenLoopRamps(openLoopConfig)
                                                        .withClosedLoopRamps(closedLoopConfig)
                                                        .withSlot0(pid0Configs);
        leftFlyConfig.MotionMagic.MotionMagicCruiseVelocity = SSC.FLY_MOTOR_MAX_VEL;
        leftFlyConfig.MotionMagic.MotionMagicAcceleration = SSC.FLY_MOTOR_ACCEL;
        leftFlyConfig.MotionMagic.MotionMagicJerk = SSC.FLY_MOTOR_JERK;

        StatusCode status = m_leftFlywheel.getConfigurator().apply(leftFlyConfig);

        if (! status.isOK()) System.out.println("Left Flywheel motor config: "
                                                +status.getDescription());
    }
    
    /**
     * 
     * @param distance Distance in meters, for scoring in the HUB
     */
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
