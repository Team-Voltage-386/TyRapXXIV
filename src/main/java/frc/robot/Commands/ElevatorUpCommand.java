package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.RumbleSubsystem;

public class ElevatorUpCommand extends Command {
    // note: can play around with this command being an onTrue or a whileTrue.
    // whileTrue allows for more human controls but is completely manual. onTrue
    // goes until it hits either limit, but has less manipulator control

    private final ElevatorSubsystem m_subsystem;
    private RumbleSubsystem m_manipRumble;
    private double m_motorVoltage;
    private LEDSubsystem m_LedSubsystem;

    public ElevatorUpCommand(ElevatorSubsystem subsystem, RumbleSubsystem manipRumble, LEDSubsystem LEDControlls) {
        this.m_motorVoltage = 10;
        this.m_subsystem = subsystem;
        this.m_manipRumble = manipRumble;
        this.m_LedSubsystem = LEDControlls;

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
        if (m_subsystem.isUpperLimitTriggered()) {
            (new SinglePulseRumble(m_manipRumble, 1, 0.3)).schedule();
            (new ClimbLimitLEDCommand(m_LedSubsystem)).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return m_subsystem.isUpperLimitTriggered();
    }

}
