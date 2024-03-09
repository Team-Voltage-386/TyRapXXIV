package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.TrapSubsystem;

public class TrapInCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private double m_motorPercentage = -1;
    private final TrapSubsystem m_subsystem;

    private int startingTrapCounter;

    public TrapInCommand(TrapSubsystem subsystem) {
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
            // When you hit the bottom of the trap retract, you should reset the trap
            // counter back to 0
            m_subsystem.resetTrapCounter();
            // If you start at the source height or lower, go down until you hit the bottom
            return m_subsystem.isMaxRetract();
        } else {
            // If you start above the source height, go down until you hit the source height
            return m_subsystem.getTrapCounter() == 0;
        }
    }

}
