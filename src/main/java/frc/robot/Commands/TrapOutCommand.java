package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.TrapSubsystem;

public class TrapOutCommand extends Command {

    private double m_motorPercentage = 1;
    private final TrapSubsystem m_subsystem;

    private int startingTrapCounter;

    public TrapOutCommand(TrapSubsystem subsystem) {
        this.m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.setTrapExtendMotor(this.m_motorPercentage);
        startingTrapCounter = m_subsystem.getTrapCounter();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setTrapExtendMotor(0.0);
    }

    @Override
    public boolean isFinished() {
        if (startingTrapCounter == 0) {
            // If you started at the bottom, go up until you hit the source height
            return (m_subsystem.getTrapCounter() == 1);
        } else if (startingTrapCounter == 1) {
            // If you started at the source height, go up until you hit the maximum value
            return (m_subsystem.getTrapCounter() == 2);
        } else {
            return m_subsystem.getTrapCounter() == 2;
        }
    }

}
