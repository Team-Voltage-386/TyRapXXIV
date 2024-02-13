package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.PneumaticSubsystem;

public class ResetPneumatics extends Command {
    PneumaticSubsystem m_pneumatics;
    Timer timer = new Timer();

    public ResetPneumatics(PneumaticSubsystem m_pneumatics) {
        this. m_pneumatics = m_pneumatics;
    }

    @Override
    public void schedule() { //very slow rn
        m_pneumatics.toggleIntake();
        Timer.delay(0.5);
        m_pneumatics.toggleStabilizer();
        Timer.delay(0.5);
        m_pneumatics.toggleLift();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
