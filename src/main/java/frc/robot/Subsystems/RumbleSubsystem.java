// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.RumblePulseCommand;

public class RumbleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CommandXboxController m_controller;
  private double m_rumbleRightLastSetValue;
  private double m_rumbleLeftLastSetValue;

  public RumbleSubsystem(CommandXboxController controller) {
    this.m_controller = controller;
    m_rumbleRightLastSetValue = 0.0;
    m_rumbleLeftLastSetValue = 0.0;

  }

  public void setRumble(RumbleType type, double value) {
    m_controller.getHID().setRumble(type, value);
    updateRumbleValues(type, value);
  }

  public void updateRumbleValues(RumbleType type, double value) {
    switch (type) {
      case kBothRumble:
        m_rumbleRightLastSetValue = value;
        m_rumbleLeftLastSetValue = value;
        break;
      case kLeftRumble:
        m_rumbleLeftLastSetValue = value;
        break;
      case kRightRumble:
        m_rumbleRightLastSetValue = value;
        break;
      default:
        assert(false);
        break;
    }
  }

   public Command setRumbleCommand(RumbleType type, double value, double timeBetweenPulses) {
      return new RumblePulseCommand(type, value, timeBetweenPulses, this);
   }

   public boolean isRumbling(RumbleType type) {
    switch (type) {
      case kBothRumble:
        return this.isRumbling(RumbleType.kLeftRumble) && this.isRumbling(RumbleType.kRightRumble);
      case kLeftRumble:
        return m_rumbleLeftLastSetValue > 0.0;
      case kRightRumble:
        return m_rumbleRightLastSetValue > 0.0;
      default:
        assert(false);
        return false;
    }
   }
}
