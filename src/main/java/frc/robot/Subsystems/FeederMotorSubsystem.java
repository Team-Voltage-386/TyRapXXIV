package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ID;

public class FeederMotorSubsystem extends SubsystemBase {
    private TalonSRX feederMotor;

    public FeederMotorSubsystem() {
        feederMotor = new TalonSRX(ID.kRollerMotorID);
        feederMotor.setInverted(true);
    }

    public void runShootFeederMotorToShoot() {
        feederMotor.set(TalonSRXControlMode.PercentOutput, 0.5);
    }

    public void runFeederMotorToLoad() {
        feederMotor.set(TalonSRXControlMode.PercentOutput, 0.2);
    }

    public void stopFeederMotor() {
        feederMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public Command stopFeederMotorCommand() {
        return runOnce(this::stopFeederMotor);
    }

    public Command runFeederMotorToLoadCommand() {
        return runOnce(this::runFeederMotorToLoad);
    }

    public Command runShootFeederMotorToShootCommand() {
        return runOnce(this::runShootFeederMotorToShoot);
    }
}
