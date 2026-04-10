// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class WaitForMillisecsCmd extends Command {
long m_startTimeMs;
long m_durationMs;

  /** Creates a new WaitForDriveCmd. */
  public WaitForMillisecsCmd(long milliseconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_durationMs = milliseconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTimeMs = System.currentTimeMillis();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((System.currentTimeMillis() - m_startTimeMs) > m_durationMs);
  }
}