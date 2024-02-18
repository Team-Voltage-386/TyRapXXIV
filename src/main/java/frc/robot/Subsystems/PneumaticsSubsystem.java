package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Pneumatics;

public class PneumaticsSubsystem extends SubsystemBase {

    // Pneumatics to lower the pickup mechanisms to the ground
    private DoubleSolenoid m_intakePneumatics;
    // Pneumatics to lift a loaded note into the shooter
    private DoubleSolenoid m_loaderPneumatics;
    // Pneumatics to latcht the loader and lock it in place or realese it from the
    // shooter
    private DoubleSolenoid m_latchPneumatics;

    private ShuffleboardTab pneumaticsTab;

    public PneumaticsSubsystem() {
        super("Pneumatics Subsystem");

        this.m_loaderPneumatics = new DoubleSolenoid(Pneumatics.kPneumaticsModule, PneumaticsModuleType.CTREPCM, 5, 4);
        this.m_intakePneumatics = new DoubleSolenoid(Pneumatics.kPneumaticsModule, PneumaticsModuleType.CTREPCM, 3, 2);
        this.m_latchPneumatics = new DoubleSolenoid(Pneumatics.kPneumaticsModule, PneumaticsModuleType.CTREPCM, 1, 0);

        pneumaticsTab = Shuffleboard.getTab(getName());

        // pneumaticsTab.add("Disabled intake");
    }

    // Brings the loader up to the shooter mechanism
    public void enableLoaderSolenoid() {
        this.m_loaderPneumatics.set(Value.kForward);
    }

    // Depowers the loader so the forces of gravity can act against it
    public void disableLoaderSolenoid() {
        this.m_loaderPneumatics.set(Value.kReverse);
    }

    // Drops the intake down in to pickup mode
    public void enableIntakeSolenoid() {
        this.m_intakePneumatics.set(Value.kForward);
    }

    // Picks up the intake back in to the frame so we can't intake a piece
    public void disableIntakeSolenoid() {
        this.m_intakePneumatics.set(Value.kReverse);
    }

    // Locks the latch so the loader can't fall back down
    public void enableLatchSolenoid() {
        this.m_latchPneumatics.set(Value.kForward);
    }

    // Unlocks the latch so the loader can fall back down to the pickup position
    public void disableLatchSolenoid() {
        this.m_latchPneumatics.set(Value.kReverse);
    }

    public Command enableLoaderSolenoidCommand() {
        return (runOnce(this::enableLoaderSolenoid));
    }

    public Command disableLoaderSolenoidCommand() {
        return (runOnce(this::disableLoaderSolenoid));
    }

    public Command enableIntakeSolenoidCommand() {
        return (runOnce(this::enableIntakeSolenoid));
    }

    public Command disableIntakeSolenoidCommand() {
        return (runOnce(this::disableIntakeSolenoidCommand));
    }

    public Command enableLatchSolenoidCommand() {
        return (runOnce(this::enableLatchSolenoid));
    }

    public Command disableLatchSolenoidCommand() {
        return (runOnce(this::disableLatchSolenoid));
    }
}
