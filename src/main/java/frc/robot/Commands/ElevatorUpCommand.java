package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorUpCommand extends Command {
    // note: can play around with this command being an onTrue or a whileTrue.
    // whileTrue allows for more human controls but is completely manual. onTrue
    // goes until it hits either limit, but has less manipulator control

    private final ElevatorSubsystem m_subsystem;
    private double m_motorVoltage;

    public ElevatorUpCommand(ElevatorSubsystem subsystem) {
        this.m_motorVoltage = 4;
        this.m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if (!m_subsystem.isUpperLimitTriggered()) {
            m_subsystem.setElevatorMotorsVoltage(m_motorVoltage);
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setElevatorMotorsVoltage(0.0);
    }

    @Override
    public boolean isFinished() {
        return m_subsystem.isUpperLimitTriggered();
    }

}
