/*package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ISC;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractIntakeToMaximumCmd extends Command {
    IntakeSubsystem m_intake;
    public RetractIntakeToMaximumCmd(IntakeSubsystem intake) {
        m_intake = intake;
    }
    @Override
    public void execute() {
        m_intake.arm_gotoRetract();
        m_intake.hopper_Halt();
        m_intake.intake_Halt();
    }
    @Override
    public boolean isFinished() {
        double armpos = m_intake.arm_getPosition();
        return(armpos < ISC.PIVOT_MOTOR_HOLD_ANGLE+0.01)&&(armpos > ISC.PIVOT_MOTOR_HOLD_ANGLE-0.01);
    }
    @Override
    public void end(boolean interrupted) {
        m_intake.arm_gotoRetract();
    }
}
*/