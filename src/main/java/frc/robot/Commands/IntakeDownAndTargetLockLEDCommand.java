// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.PickupOrchestrator;
import frc.robot.Utils.Flags;
import frc.robot.Utils.LimelightHelpers;

public class IntakeDownAndTargetLockLEDCommand extends Command {
  /** Creates a new PieceObtainedLEDCommand. */

  private LEDSubsystem m_LedSubsystem;
  private boolean m_shouldBeOn;
  private Timer m_timer;
  private Trigger m_aimWithingErrorBounds;

  public IntakeDownAndTargetLockLEDCommand(LEDSubsystem ledSubsystem, Trigger aimWithingErrorBounds) {
    this.m_LedSubsystem = ledSubsystem;
    this.m_aimWithingErrorBounds = aimWithingErrorBounds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_LedSubsystem);
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LedSubsystem.setInteriorSegmant(255, 255, 0);
    m_LedSubsystem.setExteriorSegmants(0, 0, 255);
    m_LedSubsystem.updateLEDs();
    m_shouldBeOn = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.hasElapsed(0.1)) {
      m_shouldBeOn = !m_shouldBeOn;
      m_timer.reset();
    }
    if (m_shouldBeOn) {
      m_LedSubsystem.setExteriorSegmants(241, 245, 7);
    } else {
      m_LedSubsystem.clearExteriorSegmants();
    }
    m_LedSubsystem.updateLEDs();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (PickupOrchestrator.isIntakeDown && LimelightHelpers.getTV("limelight-b")) {
      (new IntakeDownAndTargetSeenLEDCommand(m_LedSubsystem)).schedule();
    } else {
      if (PickupOrchestrator.isIntakeDown) {
        (new IntakeDownLEDCommand(m_LedSubsystem)).schedule();
      } else {
        m_LedSubsystem.clearInteriorSegmant();
        m_LedSubsystem.updateLEDs();
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!Flags.pieceState.equals(Flags.subsystemsStates.noPiece)
        || this.m_aimWithingErrorBounds.getAsBoolean());
  }
}
