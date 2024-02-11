package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ID;
import frc.robot.Constants.Shooter;
import frc.robot.Utils.Aimlock;
import frc.robot.Utils.LimelightHelpers;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax aimMotor;
    // private CANSparkMax shooterMotor;
    // private CANSparkMax rollerMotor;

    private SimpleMotorFeedforward m_aimFF;
    private SimpleMotorFeedforward m_shootFF;
    private SimpleMotorFeedforward m_rollFF;

    //if this is true it lets the handoff roller motor and the shooter rollers know to start spinning
    private boolean hasPiece = false;

    Aimlock m_aim;

    /**
     * constraints in degrees
     */
    ProfiledPIDController m_aimPID; 
    ProfiledPIDController m_shootPID;

    /**
     * desired note speed in Meters Per Second. set kShooterSpeed to this after testing
     */
    double shootSpeed;

    public ShooterSubsystem() {
        //init aim motor
        aimMotor = new CANSparkMax(ID.kShooterAimMotorID, MotorType.kBrushless);
        aimMotor.setIdleMode(IdleMode.kBrake);
        aimMotor.getEncoder().setPosition(Units.degreesToRotations(32));
        m_aimPID = new ProfiledPIDController(0.2, 0, 0, new Constraints(90, 120));
        m_aimFF = new SimpleMotorFeedforward(0.0, 0.001);
        
        //init shooter motor
        // shooterMotor = new CANSparkMax(ID.kShooterMotorID, MotorType.kBrushless);
        // shooterMotor.setIdleMode(IdleMode.kCoast);
        m_shootPID = new ProfiledPIDController(0, 0, 0, new Constraints(10, 10));
        m_shootFF = new SimpleMotorFeedforward(0.0, 0.0);

        //init roller handoff motor
        // rollerMotor = new CANSparkMax(ID.kRollerMotorID, MotorType.kBrushless);
        // rollerMotor.setIdleMode(IdleMode.kBrake);

        //get shooter speed
        shootSpeed = SmartDashboard.getNumber("ShootSpeed", Shooter.kShooterSpeed);
    }

    public void setAim(Aimlock m_aim) {
        this.m_aim = m_aim;
    }

    public void setAimPos(double n) {
        aimMotor.getEncoder().setPosition(Units.degreesToRotations(n));
    }

    /**
     * @return Shoot motor RPM
     */
    // public double getShootMotorRPM() {
    //     return shooterMotor.getEncoder().getVelocity();
    // }

    /**
     * @return Shoot motor RPM
     */
    // public double getShootMotorRPS() {
    //     return shooterMotor.getEncoder().getVelocity()/60;
    // }

    /**
     * @return how fast a note would come out of the shooter in meters per second
     */
    // public double getShooterMPS() {
    //     return getShootMotorRPS()*Math.PI*Units.inchesToMeters(4);
    // }

    /**
     * set motors spinning at their desired rpms
     */
    // public void spoolMotors() {
    //     if(hasPiece) {
    //         shooterMotor.setVoltage(ShootFF.calculate(shootSpeed) + shootPID.calculate(getShooterMPS(), shootSpeed));
    //         rollerMotor.setVoltage(RollFF.calculate(Shooter.kRollerRPM));
    //     }
    // }

    /**
     * @return The real angle at which the shooter is pointing.
     */
    public double getShooterAngle() {
        return Units.rotationsToDegrees(aimMotor.getEncoder().getPosition())/64; //for the sake of simulating a gear ratio
    }

    /**
     * aim the shooter using PID and FeedForward. pass the aimbot outputs into this
     * @param targetAngle
     */
    public void aimShooter(double targetAngle) {
        aimMotor.setVoltage(
            m_aimPID.calculate(
                getShooterAngle(),
                targetAngle)
            + m_aimFF.calculate(
                targetAngle-getShooterAngle())
        );
    }

    public void updateShootSpeed() {
        shootSpeed = SmartDashboard.getNumber("ShootSpeed", shootSpeed);
    }

    /**
     * @return desired shooter speed in MPS
     */
    public double getDesiredShooterSpeed() {
        return shootSpeed;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter angle", getShooterAngle());
        SmartDashboard.putNumber("Target angle", (m_aim.getShooterTargetAngle()));
        SmartDashboard.putNumber("vert angle speaker", Math.toDegrees(m_aim.getVerticalAngleToSpeaker()));
        SmartDashboard.putNumber("TY", LimelightHelpers.getTY("limelight-a"));
        SmartDashboard.putNumber("dist speaker", m_aim.getDistToSpeaker());
        updateShootSpeed();
        aimShooter(m_aim.getShooterTargetAngle());
    }
}
