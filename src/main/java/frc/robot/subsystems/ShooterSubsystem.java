package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem  extends SubsystemBase {
    private double targetDistance;
    private double rpm;
    public void prepareDistantShot(double distance)
    { // Distance in meters
        targetDistance = distance;
        rpm = distanceToRPM(targetDistance);
    }
    @Override
    public void periodic()
    {

    }
    private double distanceToRPM(double distance)
    {
        return distance; // TODO: make the equation, either through a lookup table, pure math, or interpolation.
    }
}
