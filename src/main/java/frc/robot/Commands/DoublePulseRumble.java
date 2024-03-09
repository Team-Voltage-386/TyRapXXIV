// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.RumbleSubsystem;

public class DoublePulseRumble extends Command {
  private Command m_doublePulseCommand;
  private boolean m_isDone;

  /** Creates a new DoublePulseRumble. */
  public DoublePulseRumble(SinglePulseRumble singlePulseCommand, double delaySeconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_doublePulseCommand = singlePulseCommand.andThen(new TimerWaitCommand(delaySeconds))
        .andThen(singlePulseCommand.copy()).andThen(Commands.runOnce(() -> m_isDone = true));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isDone = false;
    m_doublePulseCommand.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_doublePulseCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isDone;
  }
}
