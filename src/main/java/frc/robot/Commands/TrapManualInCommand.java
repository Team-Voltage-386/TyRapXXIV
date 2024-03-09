package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.TrapSubsystem;

public class TrapManualInCommand extends Command {
    private double m_motorPercentage = -1;
    private final TrapSubsystem m_subsystem;

    public TrapManualInCommand(TrapSubsystem subsystem) {
        this.m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.setTrapExtendMotor(this.m_motorPercentage);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setTrapExtendMotor(0.0);
    }

    @Override
    public boolean isFinished() {
        return m_subsystem.isMaxRetract();
    }

}
