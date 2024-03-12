// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Utils.Flags;

public class ClimbLimitLEDCommand extends Command {
  private boolean shouldBeOn; // Determines when the lights should be on
  private Timer m_timer; // The timer that determines the value of shouldBeOn
  private int i = 0;

  private LEDSubsystem m_LedSubsystem;

  /** Creates a new ClimbTopLimitLEDCommand. */
  public ClimbLimitLEDCommand(LEDSubsystem LEDSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_timer = new Timer();
    m_LedSubsystem = LEDSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_timer.reset();
    shouldBeOn = true;
    i = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (i < 5) {
      if (m_timer.hasElapsed(0.25)) {
        m_timer.reset();
        if (i % 2 == 0) {
          m_LedSubsystem.setAllLedColor(0, 0, 255);
        } else {
          m_LedSubsystem.setAllLedColor(255, 255, 0);
        }
        i++;
      }
    }
    if (i == 6) {
      shouldBeOn = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (Flags.buttonMapMode.equals(Flags.buttonMapStates.endgameMode)) {
      (new EndgameModeCommand(m_LedSubsystem)).schedule();
    } else {
      m_LedSubsystem.clearLEDs();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !shouldBeOn;
  }
}
