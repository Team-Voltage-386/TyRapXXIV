package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ID;
import frc.robot.Constants.Shooter;
// import frc.robot.Utils.Aimlock;
// import frc.robot.Utils.LimelightHelpers;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax aimMotor;
    private CANSparkMax topShooterMotor;
    private CANSparkMax bottomShooterMotor;
    // private CANSparkMax rollerMotor;
    
    DigitalInput topLimit;
    DigitalInput bottomLimit;

    private ArmFeedforward m_aimFF;
    private SimpleMotorFeedforward m_shootFF;
    private SimpleMotorFeedforward m_rollFF;

    //if this is true it lets the handoff roller motor and the shooter rollers know to start spinning
    private boolean hasPiece = false;

    // Aimlock m_aim;

    /**
     * constraints in degrees
     */
    ProfiledPIDController m_aimPID; 
    ProfiledPIDController m_shootPID;

    /**
     * desired note speed in Meters Per Second. set kShooterSpeed to this after testing
     */
    double shootSpeed;
    double hoodAngle;

    public ShooterSubsystem() {
        //init aim motor
        aimMotor = new CANSparkMax(ID.kShooterAimMotorID, MotorType.kBrushless);
        aimMotor.setIdleMode(IdleMode.kCoast); //todo change to brake after testing

        topLimit = new DigitalInput(8);
        bottomLimit = new DigitalInput(9);

        m_aimPID = new ProfiledPIDController(0.0, 0, 0.0, new Constraints(40, 90));
        m_aimFF = new ArmFeedforward(0, 0, 0);
        
        //init shooter motors
        topShooterMotor = new CANSparkMax(ID.kTopShooterMotorID, MotorType.kBrushless);
        topShooterMotor.setIdleMode(IdleMode.kCoast);
        bottomShooterMotor = new CANSparkMax(ID.kBottomShooterMotorID, MotorType.kBrushless);
        bottomShooterMotor.setIdleMode(IdleMode.kCoast);
        m_shootPID = new ProfiledPIDController(0, 0, 0, new Constraints(10, 10));
        m_shootFF = new SimpleMotorFeedforward(0.0, 0.0);

        //init roller handoff motor
        // rollerMotor = new CANSparkMax(ID.kRollerMotorID, MotorType.kBrushless);
        // rollerMotor.setIdleMode(IdleMode.kBrake);

        //get shooter speed
        shootSpeed = SmartDashboard.getNumber("ShootSpeed", Shooter.kShooterSpeed);
        hoodAngle = SmartDashboard.getNumber("hood angle", 5);
    }

    // public void setAim(Aimlock m_aim) {
    //     this.m_aim = m_aim;
    // }

    public void setAimPos(double n) {
        aimMotor.getEncoder().setPosition(Units.degreesToRotations(n));
    }

    /**
     * @return Shoot motor RPM
     */
    public double getShootMotorRPM() {
        return topShooterMotor.getEncoder().getVelocity();
    }

    /**
     * @return Shoot motor RPM
     */
    public double getShootMotorRPS() {
        return topShooterMotor.getEncoder().getVelocity()/60;
    }

    /**
     * @return how fast a note would come out of the shooter in meters per second
     */
    public double getShooterMPS() {
        return getShootMotorRPS()*Math.PI*Units.inchesToMeters(4);
    }

    /**
     * set motors spinning at their desired rpms
     */
    public void spoolMotors() {
        if(hasPiece) {
            topShooterMotor.setVoltage(m_shootFF.calculate(shootSpeed) + m_shootPID.calculate(getShooterMPS(), shootSpeed));
            // rollerMotor.setVoltage(RollFF.calculate(Shooter.kRollerRPM));
        }
    }

    /**
     * @return The real angle at which the shooter is pointing.
     */
    public double getShooterAngle() {
        return Units.rotationsToDegrees(aimMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition())/16; //for the sake of simulating a gear ratio
    }

    public void zeroShooterEncoder() {
        aimMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setZeroOffset(
            aimMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());
    }

    /**
     * aim the shooter using PID and FeedForward. pass the aimbot outputs into this
     * @param targetAngle
     */
    public void aimShooter(double targetAngle) {
        if(!getTopLimit() && !getBottomLimit()) {
            double volts = m_aimPID.calculate(
                    getShooterAngle(),
                    targetAngle)
                + m_aimFF.calculate(targetAngle, Math.toRadians(20));
                    // targetAngle-getShooterAngle());
            SmartDashboard.putNumber("volts to hood", volts);
            aimMotor.setVoltage(volts);
        }
        if(getTopLimit()) {
            aimMotor.setVoltage(0);
        }
        if(getBottomLimit()) {
            aimMotor.setVoltage(0);
        }
    }

    public void updateShootSpeed() {
        shootSpeed = SmartDashboard.getNumber("ShootSpeed", shootSpeed);
    }

    public void updateHoodAngle() {
        hoodAngle = SmartDashboard.getNumber("hood angle", hoodAngle);
    }

    public boolean getTopLimit() {
        return !topLimit.get();
    }

    public boolean getBottomLimit() {
        return !bottomLimit.get();
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
        SmartDashboard.putNumber("Raw Shooter angle", Units.rotationsToDegrees(aimMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition()));
        SmartDashboard.putBoolean("top limit", getTopLimit());
        SmartDashboard.putBoolean("bottom limit", getBottomLimit());
        // SmartDashboard.putNumber("Target angle", (m_aim.getShooterTargetAngle()));
        // SmartDashboard.putNumber("vert angle speaker", Math.toDegrees(m_aim.getVerticalAngleToSpeaker()));
        // SmartDashboard.putNumber("TY", LimelightHelpers.getTY("limelight-a"));
        // SmartDashboard.putNumber("dist speaker", m_aim.getDistToSpeaker());
        updateShootSpeed();
        aimShooter(5);
    }
}
