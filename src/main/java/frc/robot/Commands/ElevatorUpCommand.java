package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorUpCommand extends Command {

    private final ElevatorSubsystem m_subsystem;
    private double m_motorVoltage;

    public ElevatorUpCommand(ElevatorSubsystem subsystem) {
        this.m_motorVoltage = 4;
        this.m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.setElevatorMotorsVoltage(m_motorVoltage);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setElevatorMotorsVoltage(0.0); // 0.25
    }

    @Override
    public boolean isFinished() {
        return m_subsystem.isUpperLimitTriggered();
    }

}
