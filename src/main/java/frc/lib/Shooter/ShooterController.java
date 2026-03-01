package frc.lib.Shooter;

import frc.lib.Shooter.ShooterConstants;
import frc.robot.Constants.SDC;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;

public class ShooterController {
    TalonFX m_flyMotor;

    public void setFlyMotorVelocityControl(MotionMagicVelocityVoltage control) {
        m_flyMotor.setControl(control);
    }

    private void configFlyMotor() {
        var openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                       .withVoltageOpenLoopRampPeriod(ShooterConstants.SSC.Flywheel.OPEN_LOOP_RAMP_PERIOD);
                                                       //.withTorqueOpenLoopRampPeriod(0);
        var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                           .withVoltageClosedLoopRampPeriod(ShooterConstants.SSC.Flywheel.CLOSED_LOOP_RAMP_PERIOD)
                                                           .withTorqueClosedLoopRampPeriod(0);
        var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                                  .withSensorToMechanismRatio(ShooterConstants.SSC.Flywheel.GEAR_RATIO)
                                                  .withRotorToSensorRatio(1.0);
        var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(ShooterConstants.SSC.Flywheel.MOTOR_NEUTRAL_MODE)
                                                        .withInverted(ShooterConstants.SSC.Flywheel.MOTOR_INVERT)
                                                        .withPeakForwardDutyCycle(ShooterConstants.SSC.Flywheel.OUTPUT_MOTOR_LIMIT_FACTOR)
                                                        .withPeakReverseDutyCycle(-ShooterConstants.SSC.Flywheel.OUTPUT_MOTOR_LIMIT_FACTOR);
                                                        //.withDutyCycleNeutralDeadband(.001);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                                                        .withSupplyCurrentLimit(ShooterConstants.SSC.Flywheel.MOTOR_SUPPLY_CURRENT_LIMIT)
                                                        .withSupplyCurrentLimitEnable(ShooterConstants.SSC.Flywheel.ENABLE_SUPPLY_CURRENT_LIMIT)
                                                        .withStatorCurrentLimit(ShooterConstants.SSC.Flywheel.STATOR_CURRENT_LIMIT)
                                                        .withStatorCurrentLimitEnable(ShooterConstants.SSC.Flywheel.ENABLE_STATOR_CURRENT_LIMIT);
        Slot0Configs pid0Configs = new Slot0Configs().withKP(ShooterConstants.SSC.Flywheel.MOTOR_KP)
                                                     .withKI(ShooterConstants.SSC.Flywheel.MOTOR_KI)
                                                     .withKD(ShooterConstants.SSC.Flywheel.MOTOR_KD)
                                                     .withKS(ShooterConstants.SSC.Flywheel.MOTOR_KS)
                                                     .withKV(ShooterConstants.SSC.Flywheel.MOTOR_KV)
                                                     .withKA(ShooterConstants.SSC.Flywheel.MOTOR_KA);
        var flywheelConfig = new TalonFXConfiguration().withFeedback(feedbackConfig)
                                                          .withMotorOutput(motorOutputConfig)
                                                          .withCurrentLimits(currentLimitConfig)
                                                          .withOpenLoopRamps(openLoopConfig)
                                                          .withClosedLoopRamps(closedLoopConfig)
                                                          .withSlot0(pid0Configs);

        StatusCode status = m_flyMotor.getConfigurator().apply(flywheelConfig);

        if (! status.isOK()) System.out.println("Flywheel motor config: "
                                                +status.getDescription());
    }
}
