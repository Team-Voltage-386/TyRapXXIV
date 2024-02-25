package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.ExtendClimber;
import frc.robot.Commands.RetractClimber;

public class Climber extends SubsystemBase {
    private CANSparkMax liftMotor1;
    private CANSparkMax liftMotor2;
    private DigitalInput topLimit;
    private DigitalInput lowerLimit;

    private ShuffleboardTab sensorTab;

    private SimpleWidget LowDetected;
    private SimpleWidget HighDetected;

    public Climber() {
        liftMotor1 = new CANSparkMax(20, MotorType.kBrushless);
        liftMotor2 = new CANSparkMax(21, MotorType.kBrushless);
        liftMotor1.setIdleMode(IdleMode.kBrake);
        liftMotor2.setIdleMode(IdleMode.kBrake);
        liftMotor1.setInverted(true);
        liftMotor2.follow(liftMotor1);
        topLimit = new DigitalInput(4);
        lowerLimit = new DigitalInput(5);

        Constants.Controller.kDriveController.a().whileTrue(new ExtendClimber(liftMotor1, liftMotor2, topLimit)).onFalse(new SequentialCommandGroup(
                runOnce(() -> liftMotor1.setVoltage(0.0)), runOnce(() -> liftMotor2.setVoltage(0.0))));
        Constants.Controller.kDriveController.b().whileTrue(new RetractClimber(liftMotor1, liftMotor2, lowerLimit)).onFalse(new SequentialCommandGroup(
                runOnce(() -> liftMotor1.setVoltage(0.0)), runOnce(() -> liftMotor2.setVoltage(0.0))));

        sensorTab = Shuffleboard.getTab("Sensors");
        LowDetected = sensorTab.add("Low Detected", false);
        HighDetected = sensorTab.add("High Detected", false);
    }

    @Override
    public void periodic() {
        LowDetected.getEntry().setBoolean(!lowerLimit.get());
        HighDetected.getEntry().setBoolean(!topLimit.get());
    }
}
