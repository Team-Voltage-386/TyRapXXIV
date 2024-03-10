// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utils.Aimlock;
import frc.robot.Utils.Flags;
import frc.robot.Utils.Flags.subsystemsStates;

public class Robot extends TimedRobot {

    private final RobotContainer m_containter = new RobotContainer();
    private StringLogEntry commandInitializedEntry;
    private StringLogEntry commandExecutedEntry;
    private StringLogEntry commandInterruptedEntry;

    @Override
    public void robotInit() {
        DataLogManager.start();
        DataLog log = DataLogManager.getLog();
        commandInitializedEntry = new StringLogEntry(log, "CommandsInitialized");
        CommandScheduler.getInstance().onCommandInitialize((Command command) -> {
            commandInitializedEntry.append(command.getName());
        });
        commandExecutedEntry = new StringLogEntry(log, "CommandsExecuted");
        CommandScheduler.getInstance().onCommandExecute((Command command) -> {
            commandExecutedEntry.append(command.getName());
        });
        commandInterruptedEntry = new StringLogEntry(log, "CommandsInterrupted");
        CommandScheduler.getInstance()
                .onCommandInterrupt((Command interrupted, Optional<Command> interruptingCommand) -> {
                    commandInterruptedEntry.append("INT: " + interrupted.getName() + ", BY: "
                            + (interruptingCommand.isPresent() ? interruptingCommand.get().getName() : "NONE"));
                });

        m_containter.getShooter().setAimToCoastMode();
        Flags.pieceState = subsystemsStates.noPiece;
        Flags.buttonMapMode = Flags.buttonMapStates.notEndgameMode;
    }

    @Override
    public void autonomousInit() {
        m_containter.getDrivetrain().setFieldRelative(true);
        Aimlock.setDoState(Aimlock.DoState.SPEAKER);
        m_containter.clearDefaultCommand();
        m_containter.setAutoDefaultCommand();
        m_containter.getShooter().setAimToBreakMode();
        Flags.pieceState = subsystemsStates.loadedPiece; // todo
        m_containter.getAutonomousCommand().schedule();
        Flags.buttonMapMode = Flags.buttonMapStates.notEndgameMode;

    }

    @Override
    public void disabledInit() {
        m_containter.getShooter().setAimToCoastMode();\
        m_containter.m_LedSubsystem.clearLEDs();
    }

    @Override
    public void teleopInit() {
        m_containter.getDrivetrain().setFieldRelative(true);
        Aimlock.setDoState(Aimlock.DoState.SPEAKER);
        m_containter.getShooter().setAimToBreakMode();
        m_containter.setTeleDefaultCommand();
        Flags.buttonMapMode = Flags.buttonMapStates.notEndgameMode;
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

    @Override
    public void testInit() {
    }

    @Override
    public void testExit() {

    }

}
