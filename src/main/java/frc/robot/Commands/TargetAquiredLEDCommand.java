// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Utils.LimelightHelpers;

public class TargetAquiredLEDCommand extends Command {
  /** Creates a new TargetAquiredLEDCommand. */

  private LEDSubsystem m_LedSubsystem;

  public TargetAquiredLEDCommand(LEDSubsystem LedSubsystem) {
    this.m_LedSubsystem = LedSubsystem;
    addRequirements(this.m_LedSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LedSubsystem.setExteriorSegmants(0, 0, 255);
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
    return LimelightHelpers.getTV("limelight-b");
  }
}
