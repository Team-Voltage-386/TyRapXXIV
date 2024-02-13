package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.PickupSubsystem;
import frc.robot.Subsystems.PneumaticSubsystem;

public class ToggleIntake extends Command {
    PickupSubsystem m_pickup;
    PneumaticSubsystem m_pneumatics;
    static boolean stowed = true; // intake stowed at beginning of match
    Timer timer = new Timer();

    public ToggleIntake(PickupSubsystem m_pickup, PneumaticSubsystem m_pneumatics) {
        this.m_pickup = m_pickup;
        this. m_pneumatics = m_pneumatics;
    }

    @Override
    public void schedule() { //very slow rn
        if(stowed) { //if stowed
            m_pneumatics.toggleIntake();
            Timer.delay(0.5);
            m_pneumatics.toggleLift();
            Timer.delay(0.5);
            m_pneumatics.toggleStabilizer();
        }
        else { //if intake is out
            m_pneumatics.toggleIntake();
            Timer.delay(0.5);
            m_pneumatics.toggleStabilizer();
            Timer.delay(0.5);
            m_pneumatics.toggleLift();
        }
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
