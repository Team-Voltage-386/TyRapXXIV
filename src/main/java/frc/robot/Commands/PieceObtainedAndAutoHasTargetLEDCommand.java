// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Utils.Flags;
import frc.robot.Utils.LimelightHelpers;

public class PieceObtainedAndAutoHasTargetLEDCommand extends Command {

  private LEDSubsystem m_LedSubsystem;
  private Timer m_timer;

  /** Creates a new PieceObtainedAndAutoHasTarget. */
  public PieceObtainedAndAutoHasTargetLEDCommand(LEDSubsystem ledSubsystem) {
    this.m_LedSubsystem = ledSubsystem;
    m_timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_LedSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LedSubsystem.setExteriorSegmants(0, 0, 255);
    m_LedSubsystem.setInteriorSegmant(255, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (Flags.pieceState.equals(Flags.subsystemsStates.loadedPiece) && !LimelightHelpers.getTV("limelight-b")) {
      (new PieceObtainedLEDCommand(m_LedSubsystem)).schedule();
    } else {
      m_LedSubsystem.clearLEDs();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (false);
  }
}
