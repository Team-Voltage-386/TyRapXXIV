package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase {
    private DoubleSolenoid m_handOff;
    private DoubleSolenoid m_intake;
    private DoubleSolenoid m_stabilizer;
    private boolean m_liftDeployed = false;
    private boolean m_intakeDeployed = false;
    private boolean m_stabilizerDeployed = false;

    public PneumaticSubsystem() {
        //this.m_handOff = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 2, 3);
        this.m_intake = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);
        //this.m_stabilizer = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
    }

    //handoff shit

    public void enableLift() {
        if(!getLiftDeployed()) {
            this.m_handOff.set(Value.kForward);
            this.liftOut(true);
        }
    }

    public void disableLift() {
        if(getLiftDeployed()) {
            this.m_handOff.set(Value.kReverse);
            this.liftOut(false);
        }
    }

    public void toggleLift() {
        if(getLiftDeployed()) {
            disableLift();
        }
        else {
            enableLift();
        }
    }

    public boolean getLiftDeployed() {
        return m_liftDeployed;
    }

    public void liftOut(boolean isLiftOut) {
        this.m_liftDeployed = isLiftOut;
    }

    // intake shit

    public void enableIntake() {
        if(!getIntakeDeployed()) {
            this.m_intake.set(Value.kForward);
            this.intakeOut(true);
        }
    }

    public void disableIntake() {
        if(getIntakeDeployed()) {
            this.m_intake.set(Value.kReverse);
            this.intakeOut(false);
        }
    }

    public void toggleIntake() {
        if(getIntakeDeployed()) {
            disableIntake();
        }
        else {
            enableIntake();
        }
    }

    public boolean getIntakeDeployed() {
        return m_intakeDeployed;
    }

    public void intakeOut(boolean isIntakeOut) {
        this.m_intakeDeployed = isIntakeOut;
    }

    //stabilizer shit
    public void enableStabilizer() {
        if(!getStabilizerDeployed()) {
            this.m_stabilizer.set(Value.kForward);
            this.stabilizerOut(true);
        }
    }

    public void disableStabilizer() {
        if(getStabilizerDeployed()) {
            this.m_intake.set(Value.kReverse);
            this.stabilizerOut(false);
        }
    }

    public void toggleStabilizer() {
        if(getIntakeDeployed()) {
            disableStabilizer();
        }
        else {
            enableStabilizer();
        }
    }

    public boolean getStabilizerDeployed() {
        return m_stabilizerDeployed;
    }

    public void stabilizerOut(boolean isIntakeOut) {
        this.m_stabilizerDeployed = isIntakeOut;
    }
}
