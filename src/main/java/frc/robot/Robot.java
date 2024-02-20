// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private final RobotContainer m_containter = new RobotContainer();

    @Override
    public void robotInit() {
        m_containter.m_shooter.setAimToCoastMode();
    }

    @Override
    public void autonomousInit() {
        m_containter.m_shooter.setAimToBreakMode();
    }

    @Override
    public void disabledInit() {
        m_containter.m_shooter.setAimToCoastMode();
    }

    @Override
    public void teleopInit() {
        m_containter.m_shooter.setAimToBreakMode();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledPeriodic() {
        // Only needed when measuring and configuring the encoder offsets. Can comment
        // out when not used
        // m_containter.print();
    }

}
