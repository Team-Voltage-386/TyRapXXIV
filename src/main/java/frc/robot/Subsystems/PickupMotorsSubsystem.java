package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ID;

public class PickupMotorsSubsystem extends SubsystemBase {

    double goalRPM = 425;

    //CANSparkMax frontIntakeMotor;
    CANSparkMax backIntakeMotor;

    ProfiledPIDController pickupPID = new ProfiledPIDController(0.0, 0, 0.00, new Constraints(goalRPM, 1000));
    SimpleMotorFeedforward pickupFF = new SimpleMotorFeedforward(0, 0.0089);

    public PickupMotorsSubsystem() {
        // holdingPieceDetector = new DigitalInput(ID.kPieceDetector);
        backIntakeMotor = new CANSparkMax(ID.kBackPickup, MotorType.kBrushless);
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
        backIntakeMotor.setVoltage(pickupPID.calculate(getBackRealIntakeRPM(), goalRPM) + pickupFF.calculate(goalRPM));
    }

    public void runMotorsSlow() {
        backIntakeMotor.setVoltage(pickupPID.calculate(getBackRealIntakeRPM(), 50) + pickupFF.calculate(50));
    }

    public void runMotorsReverse() {
        backIntakeMotor
                .setVoltage(pickupPID.calculate(getBackRealIntakeRPM(), -goalRPM) + pickupFF.calculate(-goalRPM));
    }

    public void stopMotors() {
        backIntakeMotor.setVoltage(0);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("intake RPM", getBackRealIntakeRPM());
        // SmartDashboard.putNumber("Target RPM", goalRPM);
    }

    public Command runMotorsCommand() {
        return runOnce(this::runMotors);
    }

    public Command runMotorsSlowCommand() {
        return runOnce(this::runMotorsSlow);
    }

    public Command stopMotorsCommand() {
        return runOnce(this::stopMotors);
    }
}