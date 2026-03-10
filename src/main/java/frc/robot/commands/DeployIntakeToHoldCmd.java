package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ISC;
import frc.robot.subsystems.IntakeSubsystem;

// ONLY use this for the initial startup for the robot.
public class DeployIntakeToHoldCmd extends Command {
    IntakeSubsystem m_intake;
    public DeployIntakeToHoldCmd(IntakeSubsystem intake) {
        m_intake = intake;
    }
    @Override
    public void execute() {
        double armpos = m_intake.arm_getPosition();
        if (armpos <= ISC.PIVOT_MOTOR_HOLD_ANGLE) {
            m_intake.arm_initOverCenter();
        } else {
            m_intake.arm_gotoHold();
        }
    }
}
