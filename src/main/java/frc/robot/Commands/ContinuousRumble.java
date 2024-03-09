// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.RumbleSubsystem;

public class ContinuousRumble extends Command {
  private RumbleSubsystem m_subsystemRequirement;
  private double m_intensity;

  /** Creates a new SinglePulseRumble. */
  public ContinuousRumble(RumbleSubsystem subsystem, double intensity) {
    m_subsystemRequirement = subsystem;
    m_intensity = intensity;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystemRequirement);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystemRequirement.setRumble(RumbleType.kBothRumble, m_intensity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystemRequirement.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
