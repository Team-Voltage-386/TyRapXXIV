// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbTopLimitLEDCommand extends Command {
  private boolean shouldBeOn; // Determines when the lights should be on
  private Timer m_timer; // The timer that determines the value of shouldBeOn

  /** Creates a new ClimbTopLimitLEDCommand. */
  public ClimbTopLimitLEDCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
