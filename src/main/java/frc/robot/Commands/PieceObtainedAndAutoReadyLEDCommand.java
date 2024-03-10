// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LEDSubsystem;

public class PieceObtainedAndAutoReadyLEDCommand extends Command {

  private LEDSubsystem m_LedSubsystem;
  private Timer m_timer;
  private boolean m_shouldBeOn;

  /** Creates a new PieceObtainedAndAutoReadyLEDCommand. */
  public PieceObtainedAndAutoReadyLEDCommand(LEDSubsystem LedSubsystem) {
    this.m_LedSubsystem = LedSubsystem;
    m_timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LedSubsystem.setInteriorSegmant(255, 0, 0);
    m_shouldBeOn = false;
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.hasElapsed(0.25)) {
      m_shouldBeOn = !m_shouldBeOn;
      m_timer.reset();
    }
    if (m_shouldBeOn) {
      m_LedSubsystem.setExteriorSegmants(0, 255, 0);
    } else {
      m_LedSubsystem.clearExteriorSegmants();
    }
    m_LedSubsystem.updateLEDs();
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
}
