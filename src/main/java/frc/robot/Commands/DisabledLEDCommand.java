// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LEDSubsystem;

public class DisabledLEDCommand extends Command {
  /** Creates a new DosabledLEDCommand. */
  // Just makes the LEDs alternate from Blue to Yellow

  private final LEDSubsystem m_LedSubsystem;

  public DisabledLEDCommand(LEDSubsystem ledSubsystem) {
    this.m_LedSubsystem = ledSubsystem;
    addRequirements(this.m_LedSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (int i = 0; i < m_LedSubsystem.length(); i++) {
      if (i % 2 == 0) {
        m_LedSubsystem.setLedColor(i, 0, 0, 255);
      } else {
        m_LedSubsystem.setLedColor(i, 255, 255, 0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_LedSubsystem.clearLEDs();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
