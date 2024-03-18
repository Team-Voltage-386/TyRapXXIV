package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ID;

public class FeederMotorSubsystem extends SubsystemBase {
    private TalonSRX feederMotor;
    private boolean rapidFire;

    public FeederMotorSubsystem() {
        feederMotor = new TalonSRX(ID.kRollerMotorID);
        feederMotor.setInverted(true);
        rapidFire = false;
    }

    public void runFeederMotorToShoot() {
        feederMotor.set(TalonSRXControlMode.PercentOutput, 0.8);
    }

    public void runFeederMotorToLoad() {
        feederMotor.set(TalonSRXControlMode.PercentOutput, 0.2);
    }

    public void stopFeederMotor() {
        feederMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void enableRapidFire() {
        rapidFire = true;
    }

    public void disableRapidFire() {
        rapidFire = false;
    }

    public boolean getRapidFire() {
        return rapidFire;
    }

    public Command enableRapidFireCommand() {
        return runOnce(this::enableRapidFire);
    }

    public Command disableRapidFireCommand() {
        return runOnce(this::disableRapidFire);
    }

    public Command stopFeederMotorCommand() {
        return runOnce(this::stopFeederMotor);
    }

    public Command runFeederMotorToLoadCommand() {
        return runOnce(this::runFeederMotorToLoad);
    }

    public Command runFeederMotorToShootCommand() {
        return runOnce(this::runFeederMotorToShoot);
    }

    @Override
    public void periodic() {
        if (rapidFire) {
            runFeederMotorToShoot();
        }
    }
}
