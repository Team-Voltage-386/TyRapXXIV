package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ID;

public class PickupMotorsSubsystem extends SubsystemBase {

    double goalRPM = 500;

    DigitalInput holdingPieceDetector;

    CANSparkMax frontIntakeMotor;
    CANSparkMax backIntakeMotor;

    ProfiledPIDController pickupPID = new ProfiledPIDController(0.0, 0, 0.00, new Constraints(goalRPM, 1000));
    SimpleMotorFeedforward pickupFF = new SimpleMotorFeedforward(0, 0.0089);

    public PickupMotorsSubsystem() {
        // holdingPieceDetector = new DigitalInput(ID.kPieceDetector);
        frontIntakeMotor = new CANSparkMax(14, MotorType.kBrushless);
        frontIntakeMotor.setInverted(true);
        backIntakeMotor = new CANSparkMax(15, MotorType.kBrushless);
    }

    /**
     * returns real intake velocity, accounting for the gear ratio
     * 
     * @return
     */
    public double getFrontRealIntakeRPM() {
        return frontIntakeMotor.getEncoder().getVelocity() / 4;
    }

    /**
     * returns real intake velocity, accounting for the gear ratio
     * 
     * @return
     */
    public double getBackRealIntakeRPM() {
        return backIntakeMotor.getEncoder().getVelocity() / 4;
    }

    public void runMotors() {
        frontIntakeMotor
                .setVoltage(pickupPID.calculate(getFrontRealIntakeRPM(), goalRPM) + pickupFF.calculate(goalRPM));
        backIntakeMotor.setVoltage(pickupPID.calculate(getBackRealIntakeRPM(), goalRPM) + pickupFF.calculate(goalRPM));
    }

    public void stopMotors() {
        frontIntakeMotor.setVoltage(0);
        backIntakeMotor.setVoltage(0);
    }

    public boolean isHoldingPiece() {
        return holdingPieceDetector.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake RPM", getBackRealIntakeRPM());
        SmartDashboard.putNumber("Target RPM", goalRPM);
    }

    public Command runMotorsCommand() {
        return runOnce(this::runMotors);
    }

    public Command stopMotorsCommand() {
        return runOnce(this::stopMotors);
    }
}