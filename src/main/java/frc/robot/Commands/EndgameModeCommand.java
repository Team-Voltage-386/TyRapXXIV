// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Utils.Flags;

public class EndgameModeCommand extends Command {
  /** Creates a new EndgameModeCommand. */
  // ENDGAME mode is a pattern for the LEDs
  /*
   * In ENDGAME Mode, every 4th LED should turn RED.
   * Then, a bar with the width of 4 LEDs should smoothly scroll from side to
   * side, bouncing off each side.
   */
  private final LEDSubsystem m_LedSubsystem;

  private int m_enable;
  private boolean m_shouldAdd;
  private Timer m_timer;

  public EndgameModeCommand(LEDSubsystem LedSubsystem) {
    this.m_LedSubsystem = LedSubsystem;
    m_enable = 0;
    m_shouldAdd = true;
    this.m_timer = new Timer();
    addRequirements(this.m_LedSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_enable = 0;
    m_shouldAdd = true;
    m_timer.start();

    for (int i = 0; i < m_LedSubsystem.length(); i += 4)/* Set Every 4th LED to RED */ {
      m_LedSubsystem.setLedColor(i, 255, 0, 0);
    }
    m_LedSubsystem.updateLEDs();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (true)/* Create the scrolling bar */ {
      m_timer.reset();

      int disableOffset; // Whether to disable the LED on the Left, or the Right.
      if (m_shouldAdd) {// Checks to see if it was previously rolling to the Right or the Left, then
                        // disables the LED Behind it.
        disableOffset = -1;
      } else {
        disableOffset = 4;
      }
      if ((m_enable + disableOffset >= 0) && (m_enable + disableOffset < m_LedSubsystem.length())
          && (m_enable + disableOffset) % 4 != 0) {
        /* Disable the LED ONLY IF it shouldn't be on already. */
        m_LedSubsystem.setLedColor(m_enable + disableOffset, 0, 0, 0);
      }

      for (int i = 0; i < 4; i++) {
        m_LedSubsystem.setLedColor(i + m_enable, 255, 0, 0);
      }

      if (m_shouldAdd)/* Ether adds or subtracts from m_enable */ {
        m_enable += 1;
      } else {
        m_enable -= 1;
      }
      if (m_enable >= (m_LedSubsystem.length() - 4))/* Determines whether to make m_shouldAdd True or False */ {
        m_shouldAdd = false;
      } else if (m_enable == 0) {
        m_shouldAdd = true;
      }
    }
    // Actually change the LEDs by updating them.
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
    return (DriverStation.isDisabled() || Flags.buttonMapMode.equals(Flags.buttonMapStates.notEndgameMode));
  }
}
