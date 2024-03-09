package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorDownCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final ElevatorSubsystem m_subsystem;
    private double m_motorVoltage;

    public ElevatorDownCommand(ElevatorSubsystem subsystem) {
        this.m_motorVoltage = -10;
        this.m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if (!m_subsystem.isLowerLimitTriggered()) {
            m_subsystem.setElevatorMotorsVoltage(m_motorVoltage);
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setElevatorMotorsVoltage(0.0);
        // May need to sometimes leave it at a certain voltage if we aren't planning to
        // go down to the bottom stationary hooks
    }

    @Override
    public boolean isFinished() {
        return m_subsystem.isLowerLimitTriggered();
    }

}
