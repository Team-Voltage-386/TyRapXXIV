// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CommandXboxController m_controller;

  public RumbleSubsystem(CommandXboxController controller) {
    super("RumbleSubsystem");
    this.m_controller = controller;
  }

  public void setRumble(RumbleType type, double value) {
    m_controller.getHID().setRumble(type, value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
