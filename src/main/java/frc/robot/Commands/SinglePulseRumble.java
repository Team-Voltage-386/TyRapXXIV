// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.RumbleSubsystem;

public class SinglePulseRumble extends Command {
  private Timer m_timer;
  private RumbleSubsystem m_subsystemRequirement;
  private double m_rumbleTime;
  private double m_intensity;

  /** Creates a new SinglePulseRumble. */
  public SinglePulseRumble(RumbleSubsystem subsystem, double intensity, double timeSeconds) {
    m_subsystemRequirement = subsystem;
    m_timer = new Timer();
    m_rumbleTime = timeSeconds;
    m_intensity = intensity;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystemRequirement);
  }

  public SinglePulseRumble copy() {
    return new SinglePulseRumble(m_subsystemRequirement, m_intensity, m_rumbleTime);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_timer.reset();
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
    if (m_timer.advanceIfElapsed(m_rumbleTime)) {
      return true;
    }
    return false;
  }
}
