package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.RumbleSubsystem;

public class ElevatorDownCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final ElevatorSubsystem m_subsystem;
    private double m_motorVoltage;
    private RumbleSubsystem m_manipRumble;
    private LEDSubsystem m_LedSubsystem;

    public ElevatorDownCommand(ElevatorSubsystem subsystem, RumbleSubsystem manipRumble, LEDSubsystem LEDControls) {
        this.m_motorVoltage = -10;
        this.m_subsystem = subsystem;
        this.m_manipRumble = manipRumble;
        this.m_LedSubsystem = LEDControls;

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
        m_subsystem.setElevatorMotorsVoltage(-0.15);
        if (m_subsystem.isLowerLimitTriggered()) {
            (new SinglePulseRumble(m_manipRumble, 1, 0.3)).schedule();
            (new ClimbLimitLEDCommand(m_LedSubsystem)).schedule();
        }
        // May need to sometimes leave it at a certain voltage if we aren't planning to
        // go down to the bottom stationary hooks
    }

    @Override
    public boolean isFinished() {
        return m_subsystem.isLowerLimitTriggered();
    }

}
