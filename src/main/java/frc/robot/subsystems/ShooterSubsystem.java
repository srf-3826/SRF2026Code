package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {
    private double    m_rpmLeft;
    private double    m_rpmRight;
    private TalonFX   m_leftFlywheel;
    private TalonFX   m_rightFlywheel;
    private TalonFXS  m_leftFeedWheel;
    private TalonFXS  m_rightFeedWheel;
    private CANBus    m_shooterBus;
    private CANrange  m_canrange;

     // --- Shooter state machine States ---
    private enum ShooterState {
      IDLE,                         // Motors stopped, or placed in coast mode to spin down on their own.
      GOING_TO_TARGET_VEL,          // Spinning up (or down) to RPM
      WAITING_FOR_SINGLE_SHOT,      // ramping to speed; change state to FIRING_ONE and turn on
                                    // left feed motor and bed rollers (inward) when ready
      WAITING_FOR_CONTINUOUS_FIRE,  // ramping to speed; change state to FIRING_CONTINUOUS and turn
                                    // on bed rollers and feed wheels when ready
      READY_TO_FIRE,                // At Target RPM, but not triggered to fire
      FIRING_ONE,                   // Feed motor and bedmotors are on, waiting for "Fuel shot" sensor.
                                    // On sensor event, change state to READ_TO_FIRE, turn off feed motor
      FIRING_CONTINUOUS             // Keep all motors running, stop via manual control (Bumper release)
    }

    private enum ShootMode {
      ONE,
      CONTINUOOUS
    }

    private ShooterState  m_currentShooterState = ShooterState.IDLE;
    private double        m_targetFlywheelVel = 0.0;                  // Start out stopped 

    public ShooterSubsystem(CANBus shooterBus) {
        m_shooterBus = shooterBus;
        m_leftFlywheel   = new TalonFX(SSC.LEFT_SHOOTER_MOTOR_ID, m_shooterBus);
        m_rightFlywheel  = new TalonFX(SSC.RIGHT_SHOOTER_MOTOR_ID, m_shooterBus);
        m_leftFeedWheel  = new TalonFXS(SSC.LEFT_FEED_MOTOR_ID, m_shooterBus);
        m_rightFeedWheel = new TalonFXS(SSC.RIGHT_FEED_MOTOR_ID, m_shooterBus);
        m_canrange       = new CANrange(SSC.CANRANGE_ID, m_shooterBus);
        configFlywheels();
        configFeedMotors();
        configCANRange();
        changeStateTo(ShooterState.IDLE);
    }
    
    // Always use this method to change the state. This can then serve as a 
    // single point of contact gateway to do logging in the future.
    private void changeStateTo(ShooterState newState) {
      m_currentShooterState = newState;
    }

    public void changeFlywheelTargetVel(double vel) {
      // Filter for reasonable values, and clamp if needed
      if (vel > SSC.MAX_FLY_VEL) vel = SSC.MAX_FLY_VEL;
      if (vel < SSC.MIN_FLY_VEL) vel = SSC.MIN_FLY_VEL;
      
      // Store the target velocity in a member variable - such member data memory is
      // needed for increment and decrement adjustments in fixed 100 RPM steps, if used.
      // It is also needed to return the flywheels to the previous target vel if they have been
      // shut down for any reason.
      m_targetFlywheelVel = vel;

      /*
      * TODO: Inset code here to control the velocity PIDs. Do it for both 
      * left and right flywheels - no need to do just the left flywheel even for
      * single shots - because those are used for ranging, and continuous shooting
      * is expected shortly afterwards. 
      * 
      */

      // Now set the shooterState to GOING_TO_TARGET_VEL
      // Note that when singleShot() and fireContinuous() methods
      // call this method (it is convenient for then to use this method to 
      // spin the flywheels up to the previously set target vel when needed), 
      // they will immediately overwrite the state after return,
      // so in the end three separate states may result after this call is
      // invoked! This GOING_TO_TARGET_VEL state covers only the case where spin up
      // is desired, but no SingleShot or ContinousShooting is pending.  
      changeStateTo(ShooterState.GOING_TO_TARGET_VEL);
    }

    // These next two methods are not really needed. Instead, you can bind the same
    // changeFlywheelTargetVel() method to whatever inc and dec buttons are used, but
    // passing in the respective constants used here as arguments in the binding calls.
    public void spinUpFlywheelClose() {
        changeFlywheelTargetVel(SSC.FLY_MOTOR_NEAR_DIST_VEL);
    }

    public void spinUpFlywheelFar() {
        changeFlywheelTargetVel(SSC.FLY_MOTOR_FAR_DIST_VEL);
    }

    // The folllowing two methods could be useful in tuning - not so much for
    // competition. Maybe bind to D-Pad buttons, but only during testing - uses too
    // many button resources... 
    public void incrementFlywheelVel() {
      changeFlywheelTargetVel(m_targetFlywheelVel + 100.0);
    }

    public void decrementFlywheelVel() {
      changeFlywheelTargetVel(m_targetFlywheelVel - 100.0);
    }

    // Bind this to the button that fires a signle shot (Button Y?)
    // This entry point is the only way to initiate a single shot
    public void singleShot() {
      // First do any validit checks potentially needed. For instance, 
      // are there any states in which it would not be approriate to 
      // allow a single shot?
      // 
      // Now check if the flywheel are up to speed, and if not get that going
      if (m_currentShooterState != ShooterState.READY_TO_FIRE) {
        changeFlywheelTargetVel(m_targetFlywheelVel);
      }
      // Finally set the state to WAITING_FOR_SINGLE_SHOT. We can't go directly
      // to "startShooting" even if the flywheels are already up to speed, because
      // there may not be a ball ("fuel") under the index wheel, ready to fire.  
      // So just change the state and let the state machine handle that detail.
      changeStateTo(ShooterState.WAITING_FOR_SINGLE_SHOT);
    }
        
    // Bind this method to the button that whileHeld causes continuousShooting.
    // Renamed from fireUntilEmpty because there is no empty sensor. The only way
    // to stop is for the operator to release the button. Consider a timeout? 
    // Not a priority for first competition - but if used, maybe 25 seconds (shift
    // time) mmaximum?
    public void shootContinuous() {
      // First do any validity checks
      // Then see if the flywheels are up to speed. If not, start them up.
      if (m_currentShooterState != ShooterState.READY_TO_FIRE) {
        changeFlywheelTargetVel(m_targetFlywheelVel);
        // and change the state to WAITING_FOR_CONTINUOUS_FIRE
        changeStateTo(ShooterState.WAITING_FOR_CONTINUOUS_FIRE);
      } else {
        // The following method changes the state for you
        startShooting(ShootMode.CONTINUOOUS);
      }
    }

    public void startShooting(ShootMode shootMode) {
      // No speed filters here - assume fully vetted shooter readiness,
      // which implies you should double check your code logic before calling!
      if (shootMode == ShootMode.ONE) {
          startLeftFeedMotor();;
          changeStateTo(ShooterState.FIRING_ONE);
      } else if (shootMode == ShootMode.CONTINUOOUS) {
          startLeftFeedMotor();
          startRightFeedMotor();
          changeStateTo(ShooterState.FIRING_CONTINUOUS);
      } else {
          System.out.println("Invalid mode requested in ShooterSubsystem.startShooting()");
      }
    }

    public void startLeftFeedMotor() {
        // TODO: ensure bed rollers are on, inward
        // TODO: sent valiocity control request to left feed motor
    }
    
    public void startRightFeedMotor() {
        // TODO: ensure bed rollers are on, inward
        // TODO: send velocity PID control request to Right feed motor
    }
    
    // The stopShooting() method stops both feed motors, but leaves the shooter flywheel motors
    // running, and changes state to READY_TO_SHOOT. 
    // To stop both feed and flywheel motors (in coast mode) call shutdownShooter instead.
    // Bind the stopShooting() method to the RELEASE of whichever button is assigned to 
    // the continuous shooting function (ALT - R_Bumper).
    public void stopShooting() {
      // Are there any validity checks needed to vet this call? If not, or if checks pass, continue.
      // TODO: stop both feed motors
      // Leave flywheels unchanged
      // Let bed rollers continue to run?
      if (m_currentShooterState != ShooterState.IDLE) {
        changeStateTo(ShooterState.READY_TO_FIRE);    // Maybe a stretch, but should be harmless
      }
      // else leave state as IDLE
    }

    // Provide separate getters() for areShootersReady() and isLeftShooterReady()
    public boolean areShootersReady() {
        return (isWithinTolerance(m_rightFlywheel.getVelocity().getValueAsDouble())
                && isLeftShooterReady());
    }
    
    public boolean isLeftShooterReady() {
      return isWithinTolerance(m_leftFlywheel.getVelocity().getValueAsDouble());
    }

    // isWithinTolerance() is a helper method that uses a constant toleerance
    // (from SSC), and returns true if and only if the passed in flywheelVel velocity
    // matches the global member variable targetVel within that tolerance.
    // All units are rotations per second.
    public boolean isWithinTolerance(double vel) {
        // Inclusive check: lowerBound <= value <= upperBound
        double lowerBound  = m_targetFlywheelVel - SSC.FLY_MOTOR_VEL_TOLERANCE;
        double upperBound  = m_targetFlywheelVel + SSC.FLY_MOTOR_VEL_TOLERANCE;
        return (vel >= lowerBound && vel <= upperBound);
    }

    public boolean isFuelAtLeftShooterSensor() {
      return (m_canrange.getDistance().getValueAsDouble() <= SSC.CANRANGE_FUEL_PRESENT_THRESHOLD);
    }

    public void shutdownShooter() {
      // No vetting - just shhut everythign down immediately
      stopShooting();                // Stops feed motors
      // TODO: stop both flywheel motors
      changeStateTo(ShooterState.IDLE);
    }
    
    // runShooterStateMachine is the engine that makes everything run cleanly
    // in sequence, loop to loop, every 20 ms. Called from periodic().
    public void runShooterStateMachine() {
        switch(m_currentShooterState) {
          case IDLE:
            // Nothing to do
            break;

          case GOING_TO_TARGET_VEL:
            if (areShootersReady()) {
              changeStateTo(ShooterState.READY_TO_FIRE);
            }
            break;

          case WAITING_FOR_SINGLE_SHOT:
            if (isFuelAtLeftShooterSensor()) {
              //
              // TODO: STOP left feed motor immediately here (because flywheel may not be up to speed).
              // Now check if flywheel IS ready to shoot
              if (isLeftShooterReady()) {
                // and if so, startShooting. That method changes the state
                startShooting(ShootMode.ONE);
              }  // else not ready, so keep waiting
            } else {
              // fuel is not under the feed wheel and at the sensor, so get 
              // that action started in case necessary. Then continue to wait
              // in current state (it won't hurt to restart motor every loop until ready)
              startLeftFeedMotor();
            }
            break;
    
          case WAITING_FOR_CONTINUOUS_FIRE:
            if (areShootersReady()) {
              // yes, ready to go, so startShooting (and continue "whileHeld")
              // The startShooting method changes the state
              startShooting(ShootMode.CONTINUOOUS);
            }
            break;

          case READY_TO_FIRE:
            // Flywheels are supposed to be up to speed, but no firing is queued up, 
            // so out of an abundance of attention to detail, do a check and if the 
            // velocity is out of the acceptable range (i.e. out of TOLERANCE), 
            // resstart the velocity PID control. Doing that changes the state to 
            // GOING_TO_TARGET_VEL, but when the target velocity is reached, the 
            // state machine automatically changes it back to READY_TO_FIRE. 
            // Thus a badly controlled flywheel motor will result in a lot of
            // ping-ponging between those two states. In a well controlled system,
            // it will stay in the READY_TO_FIRE state until a shooting action is 
            // triggered.
            if (! areShootersReady()) {
              // the following method changes state to GOING_TO_TARGET_VEL
              changeFlywheelTargetVel(m_targetFlywheelVel);
            }
            break;

          case FIRING_ONE:
            if (! isFuelAtLeftShooterSensor()) {
              stopShooting();
            }
            break;

          case FIRING_CONTINUOUS:
            // Nothing to do, except one might want to enforce a time limit.
            // This state should be started only via a whileHeld button, 
            // i.e. when that button is released, firing should be stopped and 
            // the state changed back to READY_TO_FIRE (done in the method
            // to which that released button is bound to (typ. stopShooting()).
            break;

          default:
            System.out.println("Inalid state in runShooterSateMachine()" + m_currentShooterState.toString());
        }
    }

    @Override
    public void periodic(){
      // This method will be called once per scheduler run
      runShooterStateMachine();
      publishShooterData();
    }

/*
 * Insert motor config methods here, instead of the following placeholders
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
    
    private void configCANRange() {
       FovParamsConfigs fovParamsConfig = new FovParamsConfigs().withFOVCenterX(SSC.CANRANGE_FOV_CENTER_X_ANGLE)
                                                                .withFOVCenterY(SSC.CANRANGE_FOV_CENTER_Y_ANGLE)
                                                                .withFOVRangeX(SSC.CANRANGE_FOV_RANGE_X_ANGLE)
                                                                .withFOVRangeY(SSC.CANRANGE_FOV_RANGE_Y_ANGLE);
        ProximityParamsConfigs proximityParamsConfigs = new ProximityParamsConfigs().withMinSignalStrengthForValidMeasurement(SSC.CANRANGE_MINIMUM_SIGNAL)
                                                                                    .withProximityHysteresis(SSC.CANRANGE_PROXIMITY_HYSTERESIS)
                                                                                    .withProximityThreshold(SSC.CANRANGE_FUEL_PRESENT_THRESHOLD);
        ToFParamsConfigs tofParamsConfigs = new ToFParamsConfigs().withUpdateFrequency(SSC.CANRANGE_UPDATE_FREQUENCY)
                                                                  .withUpdateMode(SSC.CANRANGE_UPDATE_MODE);
        var canrangeConfig = new CANrangeConfiguration().withFovParams(fovParamsConfig)
                                                        .withProximityParams(proximityParamsConfigs)
                                                        .withToFParams(tofParamsConfigs);
        StatusCode status = m_canrange.getConfigurator().apply(canrangeConfig);
            if (! status.isOK()) System.out.println("CANrange config "
                                                    +status.getDescription());
    }  

    // This method displays shooter data to provide the drive team / programmer a window 
    // into the ShooterSubsystem operation
    private void publishShooterData() {
      m_rpmLeft = m_leftFlywheel.getVelocity().getValueAsDouble()*60;
      m_rpmRight = m_rightFlywheel.getVelocity().getValueAsDouble()*60;
      SmartDashboard.putNumber("Left RPM", m_rpmLeft);
      SmartDashboard.putNumber("Right RPM", m_rpmRight);
      var m_currentLeft = m_leftFlywheel.getSupplyCurrent().getValueAsDouble();
      var m_currentRight = m_rightFlywheel.getSupplyCurrent().getValueAsDouble();
      SmartDashboard.putNumber("Left Current", m_currentLeft);
      SmartDashboard.putNumber("Right Current", m_currentRight);
    }
}
