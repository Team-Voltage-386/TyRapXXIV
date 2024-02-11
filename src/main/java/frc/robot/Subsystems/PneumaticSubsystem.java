package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase {
    private DoubleSolenoid m_handOff;
    private DoubleSolenoid m_intake;
    private boolean m_liftDeployed = false;
    private boolean m_intakeDeployed = false;

    public PneumaticSubsystem() {
        this.m_handOff = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 2, 3);
        this.m_intake = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 4, 5);
    }

    //handoff shit

    public void enableLift() {
        this.m_handOff.set(Value.kForward);
        this.liftOut(true);
    }

    public void disableLift() {
        this.m_handOff.set(Value.kReverse);
        this.liftOut(false);
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
        this.m_intake.set(Value.kForward);
        this.intakeOut(true);
    }

    public void disableIntake() {
        this.m_intake.set(Value.kReverse);
        this.intakeOut(false);
    }

    public void toggleIntake() {
        if(getIntakeDeployed()) {
            disableLift();
        }
        else {
            enableLift();
        }
    }

    public boolean getIntakeDeployed() {
        return m_intakeDeployed;
    }

    public void intakeOut(boolean isIntakeOut) {
        this.m_intakeDeployed = isIntakeOut;
    }
}
