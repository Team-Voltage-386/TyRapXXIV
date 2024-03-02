package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Pneumatics;

public class PneumaticsSubsystem extends SubsystemBase {

    // Pneumatics to lower the pickup mechanisms to the ground
    private DoubleSolenoid m_intakePneumatics;

    public PneumaticsSubsystem() {
        super("Pneumatics Subsystem");

        this.m_intakePneumatics = new DoubleSolenoid(Pneumatics.kPneumaticsModule, PneumaticsModuleType.CTREPCM, 3, 2);
    }

    // Drops the intake down in to pickup mode
    public void enableIntakeSolenoid() {
        this.m_intakePneumatics.set(Value.kReverse);
    }

    // Picks up the intake back in to the frame so we can't intake a piece
    public void disableIntakeSolenoid() {
        this.m_intakePneumatics.set(Value.kForward);
    }

    public Command enableIntakeSolenoidCommand() {
        return (runOnce(this::enableIntakeSolenoid));
    }

    public Command disableIntakeSolenoidCommand() {
        return (runOnce(this::disableIntakeSolenoid));
    }
}
