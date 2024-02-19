package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
import frc.robot.Utils.Aimlock;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax aimMotor;
    private CANSparkMax topShooterMotor;
    private CANSparkMax bottomShooterMotor;
    private TalonSRX rollerMotor;

    DigitalInput topLimit;
    DigitalInput bottomLimit;

    private ArmFeedforward m_aimFF;
    private SimpleMotorFeedforward m_shootFF;
    private SimpleMotorFeedforward m_rollFF;

    // if this is true it lets the handoff roller motor and the shooter rollers know
    // to start spinning
    private boolean hasPiece = false;

    Aimlock m_aim;

    /**
     * constraints in degrees
     */
    ProfiledPIDController m_aimPID;
    ProfiledPIDController m_shootPID;

    /**
     * desired note speed in Meters Per Second. set kShooterSpeed to this after
     * testing
     */
    double shootSpeed;
    boolean shoot;
    // double hoodAngle;

    public ShooterSubsystem() {
        // init aim motor
        aimMotor = new CANSparkMax(ID.kShooterAimMotorID, MotorType.kBrushless);
        aimMotor.setIdleMode(IdleMode.kBrake); // todo change to brake after testing

        topLimit = new DigitalInput(8);
        bottomLimit = new DigitalInput(9);

        m_aimPID = new ProfiledPIDController(0.19, 0.0, 0.02, new Constraints(15, 90)); // 0.19
        m_aimFF = new ArmFeedforward(0.09, 0.1, 0.225); // 0.225

        // m_aimPID = new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(15, 90));
        // m_aimFF = new ArmFeedforward(0.0, 0.0, 0.0);

        // init shooter motors
        topShooterMotor = new CANSparkMax(ID.kTopShooterMotorID, MotorType.kBrushless);
        topShooterMotor.setIdleMode(IdleMode.kCoast);
        topShooterMotor.setInverted(true);
        bottomShooterMotor = new CANSparkMax(ID.kBottomShooterMotorID, MotorType.kBrushless);
        bottomShooterMotor.setIdleMode(IdleMode.kCoast);
        bottomShooterMotor.follow(topShooterMotor, false);
        m_shootPID = new ProfiledPIDController(0.25, 0, 0, new Constraints(10, 10));
        m_shootFF = new SimpleMotorFeedforward(0.0, 0.45);

        // m_shootPID = new ProfiledPIDController(0.0, 0, 0, new Constraints(10, 10));
        // m_shootFF = new SimpleMotorFeedforward(0.0, 0.0);

        // init roller handoff motor
        rollerMotor = new TalonSRX(ID.kRollerMotorID);

        // get shooter speed
        shootSpeed = Shooter.kShooterSpeed;// SmartDashboard.getNumber("ShootSpeed", Shooter.kShooterSpeed);
        // hoodAngle = SmartDashboard.getNumber("hood angle", 5);
        shoot = false;
    }

    public void hasPieceToggle() {
        hasPiece = !hasPiece;
        System.out.println(hasPiece);
    }

    public void shootToggle() {
        shoot = !shoot;
        System.out.println(shoot);
    }

    public void setAim(Aimlock m_aim) {
        this.m_aim = m_aim;
    }

    public Aimlock getAim() {
        return this.m_aim;
    }

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
        return topShooterMotor.getEncoder().getVelocity() / 60;
    }

    /**
     * @return how fast a note would come out of the shooter in meters per second
     */
    public double getShooterMPS() {
        return getShootMotorRPS() * Math.PI * Units.inchesToMeters(4);
    }

    public void driveShooterManually(double power) {
        if (power > 0 && getTopLimit()) {
        } else {
            if (power < 0 && getBottomLimit()) {
            } else
                aimMotor.setVoltage(power);
        }
    }

    double previous = 0;
    double hexRegion = 0;

    public double getHex() {
        double now = Units
                .rotationsToDegrees(aimMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());

        if (now < 50 && previous > 310 && aimMotor.getBusVoltage() > 0) {
            hexRegion++;
        }
        if (getTopLimit())
            hexRegion = 1;

        if (now > 310 && previous < 50 && aimMotor.getBusVoltage() < 0) {
            hexRegion--;
        }
        if (getBottomLimit())
            hexRegion = 0;

        SmartDashboard.putNumber("previous", previous);

        previous = now;

        SmartDashboard.putNumber("hexRegios", hexRegion);
        SmartDashboard.putNumber("now", now);
        double returnThing = now + 360 * hexRegion;
        return hexRegion < 0 ? 0 : returnThing;
    }

    /**
     * set motors spinning at their desired rpms
     */
    public void spoolMotors() {
        if (hasPiece) {
            if (shoot) {
                topShooterMotor.setVoltage(m_shootFF.calculate(shootSpeed));// + m_shootPID.calculate(getShooterMPS(),
                                                                            // shootSpeed));
                rollerMotor.set(TalonSRXControlMode.PercentOutput, -0.80);
            } else {
                topShooterMotor.setVoltage(
                        m_shootFF.calculate(shootSpeed) + m_shootPID.calculate(getShooterMPS(), shootSpeed));
            }
        } else {
            topShooterMotor.setVoltage(0);
            rollerMotor.set(TalonSRXControlMode.PercentOutput, -0.0);
        }
    }

    /**
     * @return The real angle at which the shooter is pointing.
     */
    public double getShooterAngle() {
        return getHex() / 16; // for the sake of simulating a gear ratio
    }

    public void zeroShooterEncoder() {
        aimMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setZeroOffset(
                aimMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());
    }

    /**
     * aim the shooter using PID and FeedForward. pass the aimbot outputs into this
     * 
     * @param targetAngle
     */
    public void aimShooter(double targetAngle) {
        double volts = 0;
        if (!getTopLimit() && !getBottomLimit() && Aimlock.hasTarget()) {
            volts = m_aimPID.calculate(
                    getShooterAngle(),
                    targetAngle)
                    + m_aimFF.calculate(targetAngle, targetAngle - getShooterAngle());
        }
        if (getTopLimit()) {
            volts = -0.8;
        }
        if (getBottomLimit()) {
            volts = 0.8;
        }
        if (!Aimlock.hasTarget()) {
            if (!getTopLimit()) {
                volts = 0.8;
            } else {
                volts = 0;
            }
        }

        if (volts > 0 && getTopLimit()) {
        } else {
            if (volts < 0 && getBottomLimit()) {
            } else
                aimMotor.setVoltage(volts);
        }

        SmartDashboard.putNumber("volts to hood", volts);
    }

    public void updateShootSpeed() {
        shootSpeed = SmartDashboard.getNumber("ShootSpeed", shootSpeed);
    }

    // public void updateHoodAngle() {
    // hoodAngle = SmartDashboard.getNumber("hood angle", hoodAngle);
    // }

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
        SmartDashboard.putNumber("Raw Shooter angle", getHex());
        SmartDashboard.putBoolean("top limit", getTopLimit());
        SmartDashboard.putBoolean("bottom limit", getBottomLimit());
        SmartDashboard.putNumber("shooterspeed", getShooterMPS());
        SmartDashboard.putNumber("des shoot speed", getDesiredShooterSpeed());
        // SmartDashboard.putNumber("Target angle", (m_aim.getShooterTargetAngle()));
        // SmartDashboard.putNumber("vert angle speaker",
        // Math.toDegrees(m_aim.getVerticalAngleToSpeaker()));
        // SmartDashboard.putNumber("TY", LimelightHelpers.getTY("limelight-a"));
        // SmartDashboard.putNumber("dist speaker", m_aim.getDistToSpeaker());
        updateShootSpeed();
        SmartDashboard.putNumber("target shooter angle", m_aim.getShooterTargetAngle());
        aimShooter(m_aim.getShooterTargetAngle());
        spoolMotors();
    }
}
