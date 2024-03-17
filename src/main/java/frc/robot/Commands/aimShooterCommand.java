package frc.robot.Commands;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShooterSubsystem;

public class aimShooterCommand extends Command {
    ShooterSubsystem m_shooter;

    public aimShooterCommand(ShooterSubsystem m_shooter) {
        this.m_shooter = m_shooter;
        this.m_shooter.getAimMotor().setIdleMode(IdleMode.kBrake);
        addRequirements(m_shooter);
    }

    @Override
    public void schedule() {

    }

    @Override
    public void execute() {
        m_shooter.aimShooter(m_shooter.getAim().getShooterTargetAngle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
