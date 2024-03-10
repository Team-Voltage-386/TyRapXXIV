// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LEDSubsystem;

public class AutoReadyLEDCommand extends Command {

  private LEDSubsystem m_LedSubsystem;
  private Timer m_timer;
  private boolean m_shouldBeOn = false;

  /** Creates a new AutoReadyCommand. */
  public AutoReadyLEDCommand(LEDSubsystem LedSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_LedSubsystem = LedSubsystem;
    addRequirements(this.m_LedSubsystem);
    this.m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LedSubsystem.clearExteriorSegmants();
    m_timer.start();
    m_LedSubsystem.updateLEDs();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.hasElapsed(0.1)) {
      // Determine whether the lights should be on or off
      m_shouldBeOn = !m_shouldBeOn;
      m_timer.reset();
    }
    if (m_shouldBeOn) {
      // Changes the state of the lights to on
      m_LedSubsystem.setExteriorSegmants(0, 255, 0);
    } else {
      // Changes the state of the lights to off
      m_LedSubsystem.clearExteriorSegmants();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_LedSubsystem.clearExteriorSegmants();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
