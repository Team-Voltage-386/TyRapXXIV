package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.ExtendClimber;
import frc.robot.Commands.RetractClimber;

public class Climber extends SubsystemBase {
    private CANSparkMax liftMotor1;
    private CANSparkMax liftMotor2;
    private DigitalInput topLimit;
    private DigitalInput lowerLimit;

    public Climber() {
        liftMotor1 = new CANSparkMax(20, MotorType.kBrushless);
        liftMotor2 = new CANSparkMax(21, MotorType.kBrushless);
        topLimit = new DigitalInput(4);
        lowerLimit = new DigitalInput(5);

        Constants.Controller.kDriveController.a().onTrue(new ExtendClimber(liftMotor1, liftMotor2, topLimit));
        Constants.Controller.kDriveController.b().onTrue(new RetractClimber(liftMotor1, liftMotor2, lowerLimit));
    }

}
