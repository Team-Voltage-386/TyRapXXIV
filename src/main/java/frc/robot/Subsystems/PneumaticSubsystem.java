package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Pneumatics;

public class PneumaticSubsystem extends SubsystemBase {
    
    //Pneumatics to lower the pickup mechanisms to the ground
    private DoubleSolenoid m_intakePneumatics;
    //Pneumatics to lift a loaded note into the shooter
    private DoubleSolenoid m_loaderPneumatics;
    //Pneumatics to latcht the loader and lock it in place or realese it from the shooter
    private DoubleSolenoid m_latchPneumatics;

    public PneumaticSubsystem() {
        //this.m_loaderPneumatics = new DoubleSolenoid(Pneumatics.kPneumaticsModule, PneumaticsModuleType.CTREPCM, 2, 3);
        this.m_intakePneumatics = new DoubleSolenoid(Pneumatics.kPneumaticsModule, PneumaticsModuleType.CTREPCM, 0, 1);
        //this.m_latchPneumatics = new DoubleSolenoid(Pneumatics.kPneumaticsModule, PneumaticsModuleType.CTREPCM, 6, 7);
    }

    //Brings the loader up to the shooter mechanism
    public void enableLoader() {
        this.m_loaderPneumatics.set(Value.kForward);
    }

    //Depowers the loader so the forces of gravity can act against it 
    public void disableLoader() {
        this.m_loaderPneumatics.set(Value.kReverse);
    }

    //Drops the intake down in to pickup mode
    public void enableIntake() {
        this.m_intakePneumatics.set(Value.kForward);
    }

    //Picks up the intake back in to the frame so we can't intake a piece
    public void disableIntake() {
        this.m_intakePneumatics.set(Value.kReverse);
    }

    //Locks the latch so the loader can't fall back down
    public void enableLatch() {
        this.m_latchPneumatics.set(Value.kForward);
    }

    //Unlocks the latch so the loader can fall back down to the pickup position
    public void disableLatch() {
        this.m_intakePneumatics.set(Value.kReverse);
    }
}
