package frc.robot.Commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;

public class ExtendClimber extends Command {
    private CANSparkMax motor1;
    private CANSparkMax motor2;
    private DigitalInput topLimit;

    public ExtendClimber(CANSparkMax m1, CANSparkMax m2, DigitalInput tLimit) {
        motor1 = m1;
        motor2 = m2;
        topLimit = tLimit;
    }

    @Override
    public void initialize() {
        if (topLimit.get()) {
            motor1.setVoltage(1);
        }
    }

    @Override
    public boolean isFinished() {
        return !topLimit.get();
    }

    @Override
    public void end(boolean interrupted)
    {
        motor1.setVoltage(0);
    }
}
