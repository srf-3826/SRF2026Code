package frc.lib.swerve;

import frc.lib.Sensors.Phoenix6SignalAdapters;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class SwerveModule {

    public final int m_modNum;
    private final SwerveModuleConstants m_moduleConstants;

    private final TalonFX m_steerMotor;
    private final TalonFX m_driveMotor;
    private final CANcoder m_absWheelAngleCANcoder;

    //
    // Create cached StatusSignals (Phoenix 6 recommended pattern)
    // for all data utilized by this class,
    // StatusSignals prefixed with m_ are accessed directly in this module.
    // Those without the m_ prefix are wrapped in a Phoenix6SignalAdapter 
    // helper object (instantiated in the constructor for this SwerveModule).
    //
    private final StatusSignal<Angle> steerPosSignal;
    private final StatusSignal<Double> m_sMotorOutStatusSignal;
    private final StatusSignal<Temperature> m_steerTempSignal;
    private final StatusSignal<Current> m_steerCurrentSignal;
    private final StatusSignal<Angle> absPosSignal;
    private final StatusSignal<Angle> drivePosSignal;
    private final StatusSignal<AngularVelocity> driveVelSignal;
    private final StatusSignal<Temperature> m_driveTempSignal;
    private final StatusSignal<Current> m_driveCurrentSignal;

    private final Phoenix6SignalAdapters.AngleSignal m_steerPosSignal;
    private final Phoenix6SignalAdapters.DriveSignals m_driveSignals;
    private final Phoenix6SignalAdapters.AngleSignal m_absWheelPosSignal;

    private double m_lastAngleDeg;
    private double m_velocityFeedForward;

    private final DutyCycleOut m_driveOpenLoop = new DutyCycleOut(0.0).withUpdateFreqHz(0);
    private final VelocityVoltage m_driveClosedLoop = new VelocityVoltage(0.0);
    private final PositionVoltage m_steerClosedLoop = new PositionVoltage(0.0);

    private final SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(SDC.DRIVE_KS, SDC.DRIVE_KV, SDC.DRIVE_KA);

    // Shuffleboard entries
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

        m_steerMotor = new TalonFX(moduleConstants.STEER_MOTOR_ID, swerveCanbus);
        m_driveMotor = new TalonFX(moduleConstants.DRIVE_MOTOR_ID, swerveCanbus);
        m_absWheelAngleCANcoder = new CANcoder(moduleConstants.ENCODER_ID, swerveCanbus);

        configSteerMotor();
        configDriveMotor();
        configAbsCANcoder();

        // Cache StatusSignals
        steerPosSignal          = m_steerMotor.getPosition();
        m_sMotorOutStatusSignal = m_steerMotor.getClosedLoopOutput();
        m_steerTempSignal       = m_steerMotor.getDeviceTemp();
        m_steerCurrentSignal    = m_steerMotor.getSupplyCurrent();
        drivePosSignal          = m_driveMotor.getPosition();
        driveVelSignal          = m_driveMotor.getVelocity();
        m_driveTempSignal       = m_driveMotor.getDeviceTemp();
        m_driveCurrentSignal    = m_driveMotor.getSupplyCurrent();
        absPosSignal            = m_absWheelAngleCANcoder.getAbsolutePosition();

        // Set update frequencies (CAN optimization)
        steerPosSignal.setUpdateFrequency(100);
        drivePosSignal.setUpdateFrequency(100);
        driveVelSignal.setUpdateFrequency(100);
        absPosSignal.setUpdateFrequency(10);
        m_driveTempSignal.setUpdateFrequency(4);
        m_driveCurrentSignal.setUpdateFrequency(10);
        m_steerTempSignal.setUpdateFrequency(4);
        m_steerCurrentSignal.setUpdateFrequency(10);

        // Now create StatusSignalAdapter objects that will do unit conversions
        // per request
        m_steerPosSignal = new Phoenix6SignalAdapters.AngleSignal(steerPosSignal);
        m_driveSignals = new Phoenix6SignalAdapters.DriveSignals(drivePosSignal, 
                                                                 driveVelSignal,
                                                                 SDC.TALONFX_ROT_TO_M_FACTOR);
        m_absWheelPosSignal = new Phoenix6SignalAdapters.AngleSignal(absPosSignal);

        m_steerMotor.optimizeBusUtilization();
        m_driveMotor.optimizeBusUtilization();
        m_absWheelAngleCANcoder.optimizeBusUtilization();

        // Initialize m_lastAngleDeg to the initial module state angle
        m_lastAngleDeg = getState().angle.getDegrees();

        setupModulePublishing();
    }

    // -----------------------------
    // Control Methods
    // -----------------------------
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Note: isOpenLoop is only used to determine if the wheel drive motor should
        // be driven with closed loop velocity control, or with 
        Rotation2d currentAngle2d = getAngle2d();   // reads steering encoder to get wheel direction
        desiredState.optimize(currentAngle2d);      // Compares desired angle to current angle,
                                                    // computes shortest rotation direction
    
        double desiredAngleDeg = desiredState.angle.getDegrees();
        // Add a filter here to ignore angle control unless the desired speed 
        // (before cos correction) will actually move the wheel. (Remember)
        // desired speed is divided by MAX_ROBOT_SPEED_M_PER_SEC to essentially get 
        // the motor drive output to achieve that speed (if duty cycle out were in use))
        // TunerX in manual mode was used to determine that duty cycle out of .025 was
        // needed to reliably move a wheel with no external friction (i.e. free air rotation).
        // We use a slightly smaller number (.02) to get steering output active even before 
        // the drive wheel starts moving. 
        if (Math.abs(desiredState.speedMetersPerSecond) > (SDC.MAX_ROBOT_SPEED_M_PER_SEC * 0.02)) {
            setAngle(desiredAngleDeg);
        }
        // Finish up this control method by setting the module's wheel speed. But first, 
        // in order to reduce skew when changing directions, apply a cosine correction,  
        // i.e. reduce the calculated desired speed for this module when the wheel is 
        // currently not pointing in the desired direction, proportional to the cosine 
        // of the angle error.
        // TODO: TEST how this new cosine correction affects robot maneuverability.
        desiredState.speedMetersPerSecond *= desiredState.angle.minus(currentAngle2d).getCos();
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            // Just use duty cycle out
            m_driveOpenLoop.Output =
                desiredState.speedMetersPerSecond / SDC.MAX_ROBOT_SPEED_M_PER_SEC;
            m_driveMotor.setControl(m_driveOpenLoop);
        } else {
            // Closed loop, so use velocity PID with feed forward to control output
            // based on m/sec converted to Talon Rotations / sec
            m_driveClosedLoop.Velocity =
                desiredState.speedMetersPerSecond * SDC.MPS_TO_TALONFX_RPS_FACTOR;

            // The old FF calculate(vel)) method has been deprecated
            // use calculateWithVelocities(currentVel, nextVel) instead
            m_velocityFeedForward =
                feedforward.calculateWithVelocities(getVelocityMPS(), desiredState.speedMetersPerSecond);

            m_driveMotor.setControl(
                m_driveClosedLoop.withFeedForward(m_velocityFeedForward)
            );
        }
    }

    public void setAngle(double desiredAngleDeg) {
        m_steerClosedLoop.Position =
            desiredAngleDeg * SDC.ANGLE_TO_ROTATION_FACTOR;
        m_steerMotor.setControl(m_steerClosedLoop);
        m_lastAngleDeg = desiredAngleDeg;
    }

    // -----------------------------
    // Sensor Access (Phoenix 6 optimized)
    // -----------------------------

    // Return the current wheel direction, read from the steering motor
    // Note that the steering motor uses a relative encoder - it must be
    // initialized to the current wheel direction upon startup
    private Rotation2d getAngle2d() {
        return m_steerPosSignal.asRotation2d();
    }

    // Return current Abs CANCoder direction - note that this value 
    // is Absolute, already corrected for MagnetOffset
    public double getCANcoderDeg() {
        return m_absWheelPosSignal.degrees();
    }

    // Return the current state of the module - the current wheel speed (read from
    // the drive motor) and the current wheel direction (read from the steering motor)
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getVelocityMPS(),
            getAngle2d()
        );
    }

    // Return the current distance traveled by the wheel in meters, since start up.
    public double getPositionM() {
        return m_driveSignals.positionMeters();
    }

    // Return the current wheel speed (rotations/sec, read from the drive motor,
    // converted to meters/sec traveled by the wheel
    public double getVelocityMPS() {
        return m_driveSignals.velocityMps();
    }

    // Returns the current distance traveled by the wheel, together with the
    // current direction of the wheel
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getPositionM(), getAngle2d());
    }

    // -----------------------------
    // Sync Absolute Encoder to relative TalonFX rotor sensor in steering motor
    // -----------------------------
    public void resetToAbsolute() {
        waitForCANcoder();
        m_steerMotor.setPosition(getCANcoderDeg() * SDC.ANGLE_TO_ROTATION_FACTOR);
    }


    // Ensure that we can talk to the CANCOder before syncing to the Steering motor
    private void waitForCANcoder() {
        for (int i = 0; i < 100; i++) {
            absPosSignal.refresh();
            if (absPosSignal.getStatus().isOK()) break;
            System.out.println("Mod "+m_modNum+" Waiting for CANCoder refresh issue: "
                                +absPosSignal.getStatus().getDescription());
            Timer.delay(0.005);
        }
        absPosSignal.waitForUpdate(200);
        if (! absPosSignal.getStatus().isOK()) {
            System.out.println("Mod "+m_modNum+" CANCoder waitForUpdate() issue: "
                                +absPosSignal.getStatus().getDescription());
        }
    }

    // -----------------------------
    // Configuration
    // -----------------------------

    // Cancoder
    // This CANcoder reports absolute position from [0, 1) rotations,
    // (convert to [-180, 180) via PhoenixSignalAdapters.degrees())
    // with a MagnetOffset (recorded in Constants.java) for the appropriate module
    // written here (when constructed) but not actually applied until the
    // next power cycle / boot up. CounterClockwise positive is the default,
    // but we set it anyway just to be sure.
    //
    private void configAbsCANcoder() {
        CANcoderConfiguration cfg = new CANcoderConfiguration(); 
        cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; 
        cfg.MagnetSensor.MagnetOffset = m_moduleConstants.ABS_ANG_OFFSET2D.getDegrees() 
                                        * SDC.ANGLE_TO_ROTATION_FACTOR;
        StatusCode err = m_absWheelAngleCANcoder.getConfigurator().apply(cfg);
        if (! err.isOK()) System.out.println("Mod "+m_modNum+" CANCoder Config: "
                                              +err.getDescription());
    }

    //
    // Configure steer motor (with TalonFX controller) with parameters
    // recorded in Constants.java, and passed into this SwerveModule class as
    // 
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
        MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs()
                                                        .withNeutralMode(SDC.STEER_MOTOR_NEUTRAL_MODE)
                                                        .withInverted(SDC.STEER_MOTOR_INVERT)
                                                        .withPeakForwardDutyCycle(SDC.OUTPUT_ROTATE_LIMIT_FACTOR)
                                                        .withPeakReverseDutyCycle(-SDC.OUTPUT_ROTATE_LIMIT_FACTOR);
                                                        //.withDutyCycleNeutralDeadband(.001);
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

        if (! status.isOK()) System.out.println("Mod "+m_modNum+" Steering Motor Config: "
                                                 +status.getDescription());
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
                                                        //.withDutyCycleNeutralDeadband(.001);
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
                                                     .withKA(SDC.DRIVE_KA);
        var swerveDriveConfig = new TalonFXConfiguration().withFeedback(feedbackConfig)
                                                          .withMotorOutput(motorOutputConfig)
                                                          .withCurrentLimits(currentLimitConfig)
                                                          .withOpenLoopRamps(openLoopConfig)
                                                          .withClosedLoopRamps(closedLoopConfig)
                                                          .withSlot0(pid0Configs);
        StatusCode status = m_driveMotor.getConfigurator().apply(swerveDriveConfig);

        if (! status.isOK()) System.out.println("Mod "+m_modNum+" Drive Motor Config: "
                                                +status.getDescription());
    }

    // -----------------------------------------
    // update()
    // Called from SwerveSubsystem's periodic()
    // Mostly just needs to refresh StatusSignals
    // -----------------------------------------

    public void update() {
        BaseStatusSignal.refreshAll(
                                    steerPosSignal,
                                    m_sMotorOutStatusSignal,
                                    m_steerTempSignal,
                                    m_steerCurrentSignal,
                                    absPosSignal,
                                    drivePosSignal,
                                    driveVelSignal,
                                    m_driveTempSignal,
                                    m_driveCurrentSignal
                                   );
    }

    // -----------------------------
    // Shuffleboard Publishing
    // -----------------------------
    public void setupModulePublishing() {
        ShuffleboardLayout sBE_Layout = m_moduleConstants.SBE_LAYOUT;
        // See comment above about not needing to retain these Entry keys
        /* IdsEntry (D R E)= */ sBE_Layout.add("Ids", 
                                               F.df1.format(m_moduleConstants.DRIVE_MOTOR_ID)+
                                               "  "+F.df1.format(m_moduleConstants.STEER_MOTOR_ID)+
                                               "  "+F.df1.format(m_moduleConstants.ENCODER_ID))
                                          .withPosition(0,0);
        /* absOffsetEntry = */  sBE_Layout.add("CCoff", 
                                               F.df1.format(m_moduleConstants.ABS_ANG_OFFSET2D.getDegrees()))
                                          .withPosition(0, 1);  // CC MagnetOffset
        absCANcoderDegEntry =   sBE_Layout.add("CCwd", "0")
                                          .withPosition(0,2)
                                          .getEntry();     // Current CC Abs sensor wheel direction
        steerEncoderDegEntry =  sBE_Layout.add("Swd", "0")
                                          .withPosition(0,3)
                                          .getEntry();      // Current calculated wheel direction, from motor encoder - deg
        steerSetpointDegEntry = sBE_Layout.add("Ssp", "0")
                                          .withPosition(0,4)
                                          .getEntry();      // Steering set point, i.e. current wheel direction setpoint - deg
        steerPIDOutputEntry =   sBE_Layout.add("Sout", "0")
                                          .withPosition(0,5)
                                          .getEntry();     // Steering PID motor output
        steerAmpsEntry =        sBE_Layout.add("Samps", "0")
                                          .withPosition(0,6)
                                          .getEntry();    // Steering motor current - amps
        steerTempEntry =        sBE_Layout.add("Stemp", "0")
                                          .withPosition(0,7)
                                          .getEntry();    // Steering motor temperature - Celcius
        wheelCurrSpeedEntry =   sBE_Layout.add("Dwspd", "0")
                                          .withPosition(0,8)
                                          .getEntry();    // Drive wheel angular veleocity - m/s
        wheelCurrPosEntry =     sBE_Layout.add("Ddist", "0")
                                          .withPosition(0,9)
                                          .getEntry();    // Drive wheel pos (dist) - meters
        wheelAmpsEntry =        sBE_Layout.add("Damps", "0")
                                          .withPosition(0,10)
                                          .getEntry();    // Drive motor current - amps
        wheelTempEntry =        sBE_Layout.add("Dtemp", "0")
                                          .withPosition(0,11)
                                          .getEntry();    // Drive motor temperature - Celcius
     }

    public void publishModuleData() {
        absCANcoderDegEntry.setString(F.df1.format(getCANcoderDeg()));
        steerEncoderDegEntry.setString(F.df1.format(m_steerPosSignal.degrees()));
        steerSetpointDegEntry.setString(F.df1.format(m_lastAngleDeg));
        steerPIDOutputEntry.setString(F.df2.format(m_sMotorOutStatusSignal.getValue()));
        wheelAmpsEntry.setString(F.df1.format(m_driveCurrentSignal.getValueAsDouble()));
        wheelTempEntry.setString(F.df1.format(m_driveTempSignal.getValueAsDouble()));
        steerAmpsEntry.setString(F.df1.format(m_steerCurrentSignal.getValueAsDouble()));
        steerTempEntry.setString(F.df1.format(m_steerTempSignal.getValueAsDouble()));
        wheelCurrPosEntry.setString(F.df2.format(getPositionM()));
        wheelCurrSpeedEntry.setString(F.df1.format(getVelocityMPS()));
    }

    public void stop() {
        m_driveOpenLoop.Output = 0.0;
        m_driveMotor.setControl(m_driveOpenLoop);
        m_steerMotor.stopMotor();
    }
}