package frc.lib.swerve;

import frc.lib.util.Phoenix6SignalAdapters;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;

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

    // Cached StatusSignals (Phoenix 6 recommended pattern)
    // Those prefixed with m_ are accessed directly in this module.
    // Those without _m prefix are wrapped in a Phoenix6UnitsHelper 
    // in the constructor, for access convenience in class methods.
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

    private double m_lastAngle;
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

        m_steerPosSignal = new Phoenix6SignalAdapters.AngleSignal(steerPosSignal);
        m_driveSignals = new Phoenix6SignalAdapters.DriveSignals(drivePosSignal, 
                                                                 driveVelSignal,
                                                                 SDC.TALONFX_ROT_TO_M_FACTOR);
        m_absWheelPosSignal = new Phoenix6SignalAdapters.AngleSignal(absPosSignal);

        m_steerMotor.optimizeBusUtilization();
        m_driveMotor.optimizeBusUtilization();
        m_absWheelAngleCANcoder.optimizeBusUtilization();

        m_lastAngle = getState().angle.getDegrees();

        setupModulePublishing();
    }

    // -----------------------------
    // Control Methods
    // -----------------------------
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Old approach,custom optimize
        // desiredState = SwerveOptimize.optimize(desiredState., getState().angle);
        // double desiredAngle = desiredState.angle.getDegrees();

        // New approach - use WPI optimize, which takes currentAngle as Angle2d
        Rotation2d currentAngle2d = getAngle2d();
        desiredState.optimize(currentAngle2d);
    
        double desiredAngle = desiredState.angle.getDegrees();
        // Ignore angle control unless the desired speed (before cos correction)
        // will actually move the wheel
        // TODO: replace MAX_ROBOT_SPEED_M_PER_SEC in the test expression below with a sysid 
        // (or maybe TunerX) empiracly determined value, specifically the minimum motor 
        // drive that will overcome stiction
        if (Math.abs(desiredState.speedMetersPerSecond) > (SDC.MAX_ROBOT_SPEED_M_PER_SEC * 0.01)) {
            setAngle(desiredAngle);
        }
        // Finish with the module's wheel speed control. But first, to reduce skew when 
        // changing direction, apply a cos correction, i.e. reduce the calculated desired 
        // speed when the wheel is not pointing in the desired direction, proportional 
        // to the cosine of the angle error.
        // TODO: TEST how this cos correction affects robot maneuverability.
        desiredState.speedMetersPerSecond *= desiredState.angle.minus(currentAngle2d).getCos();
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {

        if (isOpenLoop) {
            m_driveOpenLoop.Output =
                desiredState.speedMetersPerSecond / SDC.MAX_ROBOT_SPEED_M_PER_SEC;
            m_driveMotor.setControl(m_driveOpenLoop);
        } else {
            m_driveClosedLoop.Velocity =
                desiredState.speedMetersPerSecond * SDC.MPS_TO_TALONFX_RPS_FACTOR;

            m_velocityFeedForward =
                feedforward.calculate(desiredState.speedMetersPerSecond);

            m_driveMotor.setControl(
                m_driveClosedLoop.withFeedForward(m_velocityFeedForward)
            );
        }
    }

    public void setAngle(double desiredAngle) {
        m_steerClosedLoop.Position =
            desiredAngle * SDC.ANGLE_TO_ROTATION_FACTOR;
        m_steerMotor.setControl(m_steerClosedLoop);
        m_lastAngle = desiredAngle;
    }

    // -----------------------------
    // Sensor Access (Phoenix 6 optimized)
    // -----------------------------
    private Rotation2d getAngle2d() {
        return m_steerPosSignal.asRotation2d();
    }

    public double getCANcoderDeg() {
        return m_absWheelPosSignal.degrees();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getVelocityMPS(),
            getAngle2d()
        );
    }

    public double getPositionM() {
        return m_driveSignals.positionMeters();
    }

    public double getVelocityMPS() {
        return m_driveSignals.velocityMps();
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getPositionM(), getAngle2d());
    }

    // -----------------------------
    // Absolute Encoder Sync
    // -----------------------------
    public void resetToAbsolute() {
        waitForCANcoder();
        m_steerMotor.setPosition(getCANcoderDeg() * SDC.ANGLE_TO_ROTATION_FACTOR);
    }

    private void waitForCANcoder() {
        var status = absPosSignal;
        for (int i = 0; i < 100; i++) {
            if (status.getStatus().isOK()) break;
            Timer.delay(0.010);
        }
        status.waitForUpdate(200);
    }

    // -----------------------------
    // Configuration
    // -----------------------------
    private void configAbsCANcoder() {
        var cfg = new CANcoderConfiguration();
        cfg.MagnetSensor.MagnetOffset =
            m_moduleConstants.ABS_ANG_OFFSET2D.getDegrees() * SDC.ANGLE_TO_ROTATION_FACTOR;
        m_absWheelAngleCANcoder.getConfigurator().apply(cfg);
    }

    private void configSteerMotor() {
        var cfg = new TalonFXConfiguration();
        cfg.Feedback.SensorToMechanismRatio = SDC.STEER_GEAR_RATIO;
        cfg.MotorOutput.Inverted = SDC.STEER_MOTOR_INVERT;
        cfg.MotorOutput.NeutralMode = SDC.STEER_MOTOR_NEUTRAL_MODE;
        cfg.Slot0.kP = SDC.STEER_KP;
        cfg.Slot0.kI = SDC.STEER_KI;
        cfg.Slot0.kD = SDC.STEER_KD;
        cfg.ClosedLoopGeneral.ContinuousWrap = true;

        m_steerMotor.getConfigurator().apply(cfg);
    }

    private void configDriveMotor() {
        var cfg = new TalonFXConfiguration();
        cfg.Feedback.SensorToMechanismRatio = SDC.DRIVE_GEAR_RATIO;
        cfg.MotorOutput.Inverted = SDC.DRIVE_MOTOR_INVERT;
        cfg.MotorOutput.NeutralMode = SDC.DRIVE_MOTOR_NEUTRAL_MODE;
        cfg.Slot0.kP = SDC.DRIVE_KP;
        cfg.Slot0.kI = SDC.DRIVE_KI;
        cfg.Slot0.kD = SDC.DRIVE_KD;

        m_driveMotor.getConfigurator().apply(cfg);
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
        m_driveCurrentSignal.refresh();
        m_sMotorOutStatusSignal.refresh();
        m_driveTempSignal.refresh();
        m_driveCurrentSignal.refresh();
        m_steerTempSignal.refresh();
        m_steerCurrentSignal.refresh();

        absCANcoderDegEntry.setString(F.df1.format(getCANcoderDeg()));
        steerEncoderDegEntry.setString(F.df1.format(m_steerPosSignal.degrees()));
        steerSetpointDegEntry.setString(F.df1.format(m_lastAngle));
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