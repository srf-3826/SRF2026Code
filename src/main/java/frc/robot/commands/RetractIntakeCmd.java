package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ISC;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractIntakeCmd extends Command {
    IntakeSubsystem m_intake;
    public RetractIntakeCmd(IntakeSubsystem intake) {
        m_intake = intake;
    }
    @Override
    public void execute() {
        m_intake.arm_gotoHold();
        m_intake.hopper_Halt();
        m_intake.intake_Halt();
    }
}
