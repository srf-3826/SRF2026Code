package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Shooter.ShooterController;

public class ShooterSubsystem  extends SubsystemBase {
    private double targetDistance;
    private double rpm;
    private ShooterController m_shooterAssembly;

    ShooterSubsystem() {
        m_shooterAssembly = new ShooterController();
    }

    /**
     * 
     * @param distance Distance in meters, for scoring in the HUB
     */
    public void prepareDistantShot(double distance)
    {
        targetDistance = distance;
        rpm = distanceToRPM(targetDistance);
    }
    @Override
    public void periodic()
    {

    }
    public void setFlywheelVelocity(double velocity) {
        final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
        m_request.withVelocity(velocity);

        m_shooterAssembly.setFlyMotorVelocityControl(m_request);

    }
    private double distanceToRPM(double distance)
    {
        return distance; // TODO: make the equation, either through a lookup table, pure math, or interpolation.
    }
}
