// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utils.Flags;
import frc.robot.Utils.Flags.subsystemsStates;

public class Robot extends TimedRobot {

    private final RobotContainer m_containter = new RobotContainer();
    private StringLogEntry commandInitializedEntry;

    @Override
    public void robotInit() {
        DataLogManager.start();
        DataLog log = DataLogManager.getLog();
        commandInitializedEntry = new StringLogEntry(log, "CommandsInitialized");
        CommandScheduler.getInstance().onCommandInitialize((Command command) -> {
            commandInitializedEntry.append(command.getName());
        });

        m_containter.getShooter().setAimToCoastMode();
        Flags.pieceState = subsystemsStates.noPiece;
    }

    @Override
    public void autonomousInit() {
        m_containter.clearDefaultCommand();
        m_containter.getShooter().setAimToBreakMode();
        Flags.pieceState = subsystemsStates.noPiece; // todo
        m_containter.getAutonomousCommand().schedule();
    }

    @Override
    public void disabledInit() {
        m_containter.getShooter().setAimToCoastMode();
    }

    @Override
    public void teleopInit() {
        m_containter.getShooter().setAimToBreakMode();
        m_containter.setDefaultCommand();
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
