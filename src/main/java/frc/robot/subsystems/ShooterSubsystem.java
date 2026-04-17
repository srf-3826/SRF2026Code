package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {
    private IntakeSubsystem   m_intakeSubsystem;
    private double    m_rpsLeftShooter;           // measured rps value
    private double    m_rpsRightShooter;          // measured rps value
    private TalonFX   m_leftShooter;              // Names for shooter motor controllers
    private TalonFX   m_rightShooter;
    private TalonFXS  m_feeder;                   // name for feed motor controller
    private CANBus    m_shooterBus;

    // Shooter motor control support 
    private VelocityVoltage m_velocityVoltageRequest = new VelocityVoltage(0);
    private SimpleMotorFeedforward m_leftShooterFF = new SimpleMotorFeedforward(SSC.SHOOTER_KS, 
                                                                                SSC.SHOOTER_KV,
                                                                                SSC.SHOOTER_KA);
    private SimpleMotorFeedforward m_rightShooterFF = new SimpleMotorFeedforward(SSC.SHOOTER_KS, 
                                                                                 SSC.SHOOTER_KV,
                                                                                 SSC.SHOOTER_KA);

    // --- Shooter state machine enumerated States ---
    private enum ShooterState {
      IDLE,                         // Motors stopped, or placed in coast mode to spin down on their own.
      GOING_TO_TARGET_RPS,          // Spinning up (or down) to RPS. Turn off feeders and bed rollers.
      READY_TO_FIRE,                // Not really needed for post-competition, but left just  in case
                                    // we want to leave shooter flywheels running when not actually firing
      FIRING_CONTINUOUS             // Turn on bed rollers and fuel Feeders, keep all motors 
                                    // running until stopped via manual control
    }

    private ShooterState  m_currentShooterState = ShooterState.IDLE;    // Start in IDLE state
    private boolean       m_shooterTriggered = false;                   // Set when shooter is triggered to start
                                                                        // cleared when shooter is manually stopped
    private double        m_shooterTargetRps = SSC.SHOOTER_FAR_DIST_RPS;  // Start with target rps 0, i.e. motor stopped
    private double        count = 0.0;                                  // Temp variable used to reduce 
                                                                        // debug trace statement output frequency
                                                                        // while shooter is spinning up.
    // This is the constructor for a ShooterSubsystem
    // Inputs are the CANBus to use, and a handle to the IntakeSubsystem
    // so thet the bed rollers can be syncronized with the ShooterSubsystem.
    public ShooterSubsystem(CANBus shooterBus, IntakeSubsystem intakeSubsystem) {
        m_shooterBus = shooterBus;
        m_intakeSubsystem = intakeSubsystem;
        m_leftShooter   = new TalonFX(SSC.LEFT_SHOOTER_MOTOR_ID, m_shooterBus);
        m_rightShooter  = new TalonFX(SSC.RIGHT_SHOOTER_MOTOR_ID, m_shooterBus);
        m_feeder = new TalonFXS(SSC.FEED_MOTOR_ID, m_shooterBus);
        configShooterMotor( m_leftShooter, InvertedValue.Clockwise_Positive);
        configShooterMotor( m_rightShooter, InvertedValue.CounterClockwise_Positive);
        configFeedMotor(m_feeder, SSC.FEED_MOTOR_INVERT);
        changeStateTo(ShooterState.IDLE);   // State is already decalred IDLE, but 
                                            // set to IDLE here anyway to get benefit of
                                            // state change tracking.
    }
    
    // Always use this method to change the state. This can then serve as a 
    // single gateway to do state change tracking, and logging in the future.
    private void changeStateTo(ShooterState newState) {
      // Temp debug trace statement:
      System.out.println("State changed from "+m_currentShooterState.toString()+" to "+newState.toString());
      m_currentShooterState = newState;
    }

    //****************************************************************
    //
    // Public methods providing UI Control of the ShooterSubsystem
    //
    //****************************************************************
    
    // The ShootContinuous method is bound to a game controller button,
    // Shooting will continue until eihter the same game controller button 
    // is released (for competition) or until a different game controller 
    // button, which stops the shooting.
    public void shootContinuous() {
      // Debug tracer statement:
      System.out.println("Shoot continuous called. State = "+m_currentShooterState.toString());

      // This is the only place m_shooterTriggered is set to true:
      m_shooterTriggered = true;

      // See if the shooter wheels are up to speed. If not, start them up.
      if (m_currentShooterState != ShooterState.READY_TO_FIRE) { 
        changeShooterTargetRps(m_shooterTargetRps);     // CHanges state to ShooterState.GOING_TO_TARGET_RPS
        changeStateTo(ShooterState.GOING_TO_TARGET_RPS); // So fix that
      } else {
        // The shooter is already up to speed, so startShooting() (which changes the state).
        startShooting();
      }
    }

    public void startShooting() {
      // No filters here - only called if shooter is READY_TO_FIRE.
      // What needs doing here is to turn on the bed roller motor:
      m_intakeSubsystem.hopper_FeedFuelToShooter();
      // and the feed motor:
      startFeedMotor();
      // and change the state appropriately:
      changeStateTo(ShooterState.FIRING_CONTINUOUS);
    }
    
    // The stopShooting() method stops the feed and hpper bed motors, 
    // but leaves the shooter motors running, just changing state to READY_TO_FIRE. 
    // To stop both feed and shooter motors call shutdownShooter instead (which
    // in turn calls this method).
    public void stopShooting() {
      m_shooterTriggered = false;
      m_feeder.stopMotor();
      m_intakeSubsystem.hopper_Halt();
      // Leave flywheels unchanged, but change state to READY_TO_FIRE
      // unless the state is currently IDLE
      if (m_currentShooterState != ShooterState.IDLE) {
        changeStateTo(ShooterState.READY_TO_FIRE);
      }
    }

    // The shutdownShooter method not only stops shooting, but also stops
    // all motors, including the shooter flywheels. It is called only from 
    // a bound button (i.e. there are no timeouts - to stop shooting is a manual
    // input).
    public void shutdownShooter() {
      // No vetting - just shhut everything down immediately
      stopShooting();                     // stop feed and hopper bed mootors, 
                                          // and also clear m_shooterTriggered flag
      m_leftShooter.stopMotor();          // Now stop the shooter motors
      m_rightShooter.stopMotor();
      changeStateTo(ShooterState.IDLE);   // And change state to IDLE
    }
   
       // These next two methods facilitate selecting near or far distances
    // from shooter to goal.
    public void spinUpShooterClose() {
        changeShooterTargetRps(SSC.SHOOTER_NEAR_DIST_RPS);
    }

    public void spinUpShooterFar() {
        changeShooterTargetRps(SSC.SHOOTER_FAR_DIST_RPS);
    }

    // The folllowing two methods could be useful in tuning - not so much for
    // competition. Maybe bind to D-Pad buttons, but only during testing - uses too
    // many button resources... Each call bumps the target setpoint RPS up or down by 100 RPM
    public void incrementShooterVel() {
      changeShooterTargetRps(m_shooterTargetRps + 100.0/60);
    }

    public void decrementShooterVel() {
      changeShooterTargetRps(m_shooterTargetRps - 100.0/60);
    }
    
    //============================================================
    //
    // private methods in support of ShooterSubsystem operation
    //
    //============================================================

    // This method is called when it is deisred to either start up 
    // the shooter wheels (passing in either a new or the last active 
    // m_shooterTargetRps), or to change the target velocity (in rps) 
    // of shooter wheels that are already running. For example, to change
    // between a near or far shot, or to incrment or 
    // decrement the current shooter velocity for tuning purposes.
    // This is the only method where control directives are issued to the 
    // shooter wheel motor controllers.
    private void changeShooterTargetRps(double newRps) {
      double leftVelFeedforward;
      double rightVelFeedforward;
      // Filter for reasonable values, and clamp between min and max if needed
      // Temp debug trace statement:
      System.out.println("New Target Shooter Rps before clamping: " + newRps);
      if (newRps > SSC.MAX_SHOOTER_RPS) newRps = SSC.MAX_SHOOTER_RPS;
      if (newRps < SSC.MIN_SHOOTER_RPS) newRps = SSC.MIN_SHOOTER_RPS;
      // Temp debug trace statement:
      System.out.println("New Target Shooter Velocity after clamping: " + newRps);
      // Store the target velocity in a member variable to enable increment and 
      // decrement adjustments and retained the adjusted target.
      m_shooterTargetRps = newRps;

      leftVelFeedforward = m_leftShooterFF.calculateWithVelocities(m_rpsLeftShooter, m_shooterTargetRps);
      rightVelFeedforward = m_rightShooterFF.calculateWithVelocities(m_rpsRightShooter, m_shooterTargetRps);
      // Temp debug trace statement
      System.out.println("Calculated Shooter Feedforwards - left: "+leftVelFeedforward+" right: "+rightVelFeedforward);
      m_leftShooter.setControl(m_velocityVoltageRequest.withVelocity(m_shooterTargetRps)
                                                       .withFeedForward(leftVelFeedforward));
      m_rightShooter.setControl(m_velocityVoltageRequest.withVelocity(m_shooterTargetRps)
                                                        .withFeedForward(rightVelFeedforward));
      // Temp debug trace statement:
      System.out.println("Readback Shooter Feedforwards - left: "
                          +m_leftShooter.getClosedLoopFeedForward()
                          +" right: "
                          +m_rightShooter.getClosedLoopFeedForward());
    }
    
    // areShootersReady chacks both shooters for Rps being within tolerance of 
    // the target Rps. Class member variables m_rpsLeftShooter and m_rpsRightShooter
    // are read and stored once per loop by publishShooterData(), so no need to 
    // re-read them again here.
    private boolean areShootersReady() {
      return (isWithinTolerance(m_rpsRightShooter)
              &&
              isWithinTolerance(m_rpsLeftShooter));
    }

    // isWithinTolerance() is a helper method that uses a specified Rps tolerance
    // (from SSC - ShooterSubsystemConstants), and returns true if and only if 
    // the passed in current shooter velocity matches the m_shooterTargetRps 
    // within that tolerance.
    private boolean isWithinTolerance(double rps) {
        // Inclusive check: lowerBound <= value <= upperBound
        double lowerBound  = m_shooterTargetRps - SSC.SHOOTER_RPS_TOLERANCE;
        double upperBound  = m_shooterTargetRps + SSC.SHOOTER_RPS_TOLERANCE;
        return (rps >= lowerBound && rps <= upperBound);
    }

    private void startFeedMotor() {
        VelocityVoltage request = new VelocityVoltage(SSC.FEED_MOTOR_TARGET_RPS);
        m_feeder.setControl(request);
    }

    // runShooterStateMachine is the engine that makes everything run cleanly
    // in sequence, loop to loop, every 20 ms. It is called from periodic().
    private void runShooterStateMachine() {
        switch(m_currentShooterState) {
          case IDLE:
            // Nothing to do
            break;

          case GOING_TO_TARGET_RPS:
            if (areShootersReady()) {
              changeStateTo(ShooterState.READY_TO_FIRE);
            } else if (count++ % 10.0 < .001) {     // Decrease debug trace output frequency by 10
              // Temp debug trace statement
              System.out.println("Shooter not ready. Target rps = "+m_shooterTargetRps+
                                 " Curr Left rps = "+m_rpsLeftShooter+
                                 " Curr Right rps = "+m_rpsRightShooter);
            }
            break;

          case READY_TO_FIRE:
            // Shooters are up to speed. 
            // If firing is queued up, switch to firing.
            // Otherwise, just remain in this state until stopped  by manual input. 
            // While here, check the velocity - if out of TOLERANCE, 
            // repeat the changeShooterTargetRps request, which 
            // while not really a change, will generate debug trace 
            // outputs to alert us to less than ideal control.
            if (m_shooterTriggered) {
              startShooting();
            } else if (! areShootersReady()) {
              // the following method changes state to GOING_TO_TARGET_RPS
              changeShooterTargetRps(m_shooterTargetRps);
            }
            break;

          case FIRING_CONTINUOUS:
            // Nothing to do in this state except leave everthing running until
            // stopped by manual input (handled by stopShooting() method, which changes
            // the state)
            break;

          default:
            System.out.println("Inalid state in runShooterSateMachine()" + m_currentShooterState.toString());
        }
    }

    // The periodic method is called once per loop
    @Override
    public void periodic(){
      publishShooterData();         // Call publish first so all variables are freshly read       
      runShooterStateMachine();     // before running the state machine.
    }

  /*
   * Motor config methods
   */
    private void configShooterMotor(TalonFX motor, InvertedValue invertDirection) {
        var openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                       .withVoltageOpenLoopRampPeriod(SSC.SHOOTER_OPEN_LOOP_RAMP_PERIOD);
                                                       //.withTorqueOpenLoopRampPeriod(0);
        var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                           .withVoltageClosedLoopRampPeriod(SSC.SHOOTER_CLOSED_LOOP_RAMP_PERIOD)
                                                           .withTorqueClosedLoopRampPeriod(0);
        /*
        This config isnt needed if just using the rotor sensor
        var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                                  .withSensorToMechanismRatio(1)
                                                  .withRotorToSensorRatio(SSC.SHOOTER_GEAR_RATIO);
       */
        var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(SSC.SHOOTER_NEUTRAL_MODE)
                                                        .withInverted(invertDirection)
                                                        .withPeakForwardDutyCycle(SSC.SHOOTER_OUTPUT_MOTOR_LIMIT)
                                                        .withPeakReverseDutyCycle(-SSC.SHOOTER_OUTPUT_MOTOR_LIMIT);
                                                        //.withDutyCycleNeutralDeadband(.001);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                                                        .withSupplyCurrentLimit(SSC.SHOOTER_SUPPLY_CURRENT_LIMIT)
                                                        .withSupplyCurrentLimitEnable(SSC.SHOOTER_ENABLE_SUPPLY_CURRENT_LIMIT)
                                                        .withStatorCurrentLimit(SSC.SHOOTER_STATOR_CURRENT_LIMIT)
                                                        .withStatorCurrentLimitEnable(SSC.SHOOTER_ENABLE_STATOR_CURRENT_LIMIT);
        Slot0Configs pid0Configs = new Slot0Configs().withKP(SSC.SHOOTER_KP)
                                                     .withKI(SSC.SHOOTER_KI)
                                                     .withKD(SSC.SHOOTER_KD)
                                                     .withKS(SSC.SHOOTER_KS)
                                                     .withKV(SSC.SHOOTER_KV)
                                                     .withKA(SSC.SHOOTER_KA)
                                                     .withKG(SSC.SHOOTER_KG);
        var flyMotorConfig = new TalonFXConfiguration().withMotorOutput(motorOutputConfig)
                                                      .withCurrentLimits(currentLimitConfig)
                                                      .withOpenLoopRamps(openLoopConfig)
                                                      .withClosedLoopRamps(closedLoopConfig)
                                                      .withSlot0(pid0Configs);
                                                      
        StatusCode status = motor.getConfigurator().apply(flyMotorConfig);

        if (! status.isOK()) System.out.println("Shooter motor "+motor.getDeviceID()+" config error: "
                                                +status.getDescription());
    }

    private void configFeedMotor(TalonFXS motor, InvertedValue invertDirection) {
        OpenLoopRampsConfigs openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                       .withVoltageOpenLoopRampPeriod(SSC.FEED_OPEN_LOOP_RAMP_PERIOD);
                                                       //.withTorqueOpenLoopRampPeriod(0);
        ClosedLoopRampsConfigs closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                           .withVoltageClosedLoopRampPeriod(SSC.FEED_CLOSED_LOOP_RAMP_PERIOD)
                                                           .withTorqueClosedLoopRampPeriod(0);
        /*
        This config is not needed if just using the rotor sensor
        FeedbackConfigs feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                                  .withSensorToMechanismRatio(SSC.FEED_GEAR_RATIO)
                                                  .withRotorToSensorRatio(1.0);
        */
        MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs().withNeutralMode(SSC.FEED_MOTOR_NEUTRAL_MODE)
                                                        .withInverted(invertDirection)
                                                        .withPeakForwardDutyCycle(SSC.FEED_MOTOR_OUTPUT_LIMIT)
                                                        .withPeakReverseDutyCycle(-SSC.FEED_MOTOR_OUTPUT_LIMIT);
                                                        //.withDutyCycleNeutralDeadband(.001);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(SSC.FEED_SUPPLY_CURRENT_LIMIT)
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
        var feedMotorConfig = new TalonFXSConfiguration().withMotorOutput(motorOutputConfig)
                                                          .withCurrentLimits(currentLimitConfig)
                                                          .withOpenLoopRamps(openLoopConfig)
                                                          .withClosedLoopRamps(closedLoopConfig)
                                                          .withSlot0(pid0Configs)
                                                          .withCommutation(commutationConfigs);
        StatusCode status = motor.getConfigurator().apply(feedMotorConfig);
        if (! status.isOK()) System.out.println("Feed motor "+motor.getDeviceID()+" config error: "
                                                +status.getDescription());
    }

    // This method displays shooter data to provide the drive team / programmer a window 
    // into the ShooterSubsystem operation
    private void publishShooterData() {
      m_rpsLeftShooter = m_leftShooter.getVelocity().getValueAsDouble();
      m_rpsRightShooter = m_rightShooter.getVelocity().getValueAsDouble();
      SmartDashboard.putNumber("Left RPS", m_rpsLeftShooter);
      SmartDashboard.putNumber("Right RPS", m_rpsRightShooter);
      var m_currentLeft = m_leftShooter.getSupplyCurrent().getValueAsDouble();
      var m_currentRight = m_rightShooter.getSupplyCurrent().getValueAsDouble();
      SmartDashboard.putNumber("Left Current", m_currentLeft);
      SmartDashboard.putNumber("Right Current", m_currentRight);
    }
}
