package frc.robot.Commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;

public class RetractClimber extends Command {
    private CANSparkMax motor1;
    private CANSparkMax motor2;
    private DigitalInput lowLimit;

    public RetractClimber(CANSparkMax m1, CANSparkMax m2, DigitalInput lLimit) {
        motor1 = m1;
        motor2 = m2;
        motor2.follow(motor1);
        lowLimit = lLimit;
    }

    @Override
    public void initialize() {
        if (lowLimit.get()) {
            motor1.setVoltage(-0.5);
        }
    }

    @Override
    public boolean isFinished() {
        if (!lowLimit.get()) {
            motor1.setVoltage(0);
            return true;
        }
        return false;
    }

}
