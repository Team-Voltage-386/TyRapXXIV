// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LEDSubsystem;

public class ClimbLimitSwitchTriggeredLEDCOmmand extends Command {
  private boolean shouldBeOn; // Determines when the lights should be on
  private Timer m_timer; // The timer that determines the value of shouldBeOn
  private LEDSubsystem m_LedSubsystem;
  private boolean m_isDone;
  private int i;

  /** Creates a new ClimbTopLimitLEDCommand. */
  public ClimbLimitSwitchTriggeredLEDCOmmand(LEDSubsystem ledSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_timer = new Timer();
    this.m_LedSubsystem = ledSubsystem;
    addRequirements(this.m_LedSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_isDone = false;
    i = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Make i = 0; if i is even, turn the lights on. Else, turn them off.
  // Keep i < 6
  @Override
  public void execute() {
    if (i < 5) {
      if (m_timer.hasElapsed(0.25)) {
        m_timer.reset();
        i++;
      }
      if (i % 2 == 0) {
        m_LedSubsystem.setAllLedColor(255, 0, 0);
      } else {
        m_LedSubsystem.setAllLedColor(0, 0, 0);
      }
    }
    if (i == 6) {
      m_isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_timer.reset();
    m_LedSubsystem.clearLEDs();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isDone;

  }
}
