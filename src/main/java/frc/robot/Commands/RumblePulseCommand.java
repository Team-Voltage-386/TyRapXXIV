// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import frc.robot.Subsystems.RumbleSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RumblePulseCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final RumbleSubsystem m_subsystem;

  private Timer m_timer;
  private RumbleType m_rumbleType;
  private double m_value;
  private double m_timeBetweenPulses;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RumblePulseCommand(RumbleType rumbleType, double value, double timeBetweenPulses, RumbleSubsystem subsystem) {
    m_subsystem = subsystem;
    m_timer = new Timer();
    this.m_rumbleType = rumbleType;
    this.m_value = value;
    this.m_timeBetweenPulses = timeBetweenPulses;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.hasElapsed(m_timeBetweenPulses)) {
      if (m_subsystem.isRumbling(m_rumbleType)) {
        // Stop rumbling, we want to stop the pulse that is currently running
        m_subsystem.setRumble(this.m_rumbleType, 0);
      } else {
        // Start rumbling, we want to start the pulse
        m_subsystem.setRumble(this.m_rumbleType, this.m_value);
      }
      m_timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setRumble(this.m_rumbleType, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
