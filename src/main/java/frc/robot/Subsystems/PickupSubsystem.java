package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ID;

public class PickupSubsystem extends SubsystemBase {

    double goalRPM = 200;

    // CANSparkMax frontIntakeMotor;
    CANSparkMax backIntakeMotor;

    ProfiledPIDController pickupPID = new ProfiledPIDController(0.001, 0, 0.0, new Constraints(goalRPM, 1000));
    SimpleMotorFeedforward pickupFF = new SimpleMotorFeedforward(0, 0.0085);

    public PickupSubsystem () {
        // frontIntakeMotor = new CANSparkMax(0, MotorType.kBrushless);
        backIntakeMotor = new CANSparkMax(ID.kShooterAimMotorID, MotorType.kBrushless);
    }

    /**
     * returns real intake velocity, accounting for the gear ratio
     * @return
     */
    // public double getFrontRealIntakeRPM() {
    //     return frontIntakeMotor.getEncoder().getVelocity()/4;
    // }

    /**
     * returns real intake velocity, accounting for the gear ratio 
     * @return
     */
    public double getBackRealIntakeRPM() {
        return backIntakeMotor.getEncoder().getVelocity()/4;
    }


    public void setMotors() {
        // frontIntakeMotor.setVoltage(pickupPID.calculate(getFrontRealIntakeRPM(), goalRPM) + pickupFF.calculate(goalRPM));
        backIntakeMotor.setVoltage(pickupPID.calculate(getBackRealIntakeRPM(), goalRPM) + pickupFF.calculate(goalRPM));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake RPM", getBackRealIntakeRPM());
        SmartDashboard.putNumber("Target RPM", goalRPM);
        setMotors();
    }
}
