// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Utils.Flags;

public class PieceObtainedLEDCommand extends Command {
  /** Creates a new PieceObtainedLEDCommand. */

  private LEDSubsystem m_LedSubsystem;
  private Timer m_timer;
  private boolean shouldBeOn;

  public PieceObtainedLEDCommand(LEDSubsystem ledSubsystem) {
    this.m_LedSubsystem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_LedSubsystem);
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LedSubsystem.setInteriorSegmant(255, 0, 0);
    m_LedSubsystem.updateLEDs();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_LedSubsystem.clearInteriorSegmant();
    m_LedSubsystem.updateLEDs();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Flags.pieceState.equals(Flags.subsystemsStates.noPiece));
  }
}
