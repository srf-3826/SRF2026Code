/*package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ISC;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntakeCmd extends Command {
    IntakeSubsystem m_intake;
    public DeployIntakeCmd(IntakeSubsystem intake) {
        m_intake = intake;
    }
    @Override
    public void execute() {
        //double armpos = m_intake.arm_getPosition();
        //if (armpos <= ISC.PIVOT_MOTOR_HOLD_ANGLE) {
           // m_intake.arm_initOverCenter();
       // } else {
           // m_intake.arm_gotoFloorPickup();
        //}
        m_intake.hopper_FeedFuelToShooter();
       // m_intake.intake_IntakeFuel();
    }
    @Override
    public boolean isFinished() {
       // double armpos = m_intake.arm_getPosition();
        return(armpos < ISC.PIVOT_MOTOR_FLOOR_ANGLE+0.01)&&(armpos > ISC.PIVOT_MOTOR_FLOOR_ANGLE-0.01);
    }
    @Override
    public void end(boolean interrupted) {
       // m_intake.arm_gotoFloorPickup();
    }
}
*/