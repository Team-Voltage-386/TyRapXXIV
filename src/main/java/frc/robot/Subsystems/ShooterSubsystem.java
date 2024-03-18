package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ID;
import frc.robot.Constants.Shooter;
import frc.robot.Utils.Aimlock;
import frc.robot.Utils.BellController;
import frc.robot.Utils.Flags;
import frc.robot.Utils.ParabolicController;
import frc.robot.Utils.Aimlock.DoState;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax aimMotor;
    private CANSparkMax topShooterMotor;
    private CANSparkMax bottomShooterMotor;
    private RelativeEncoder relativeEncoder;

    DigitalInput topLimit;
    DigitalInput bottomLimit;

    private ArmFeedforward m_aimFF;
    private ProfiledPIDController m_aimPID;
    private SimpleMotorFeedforward m_topShootFF;
    private ProfiledPIDController m_topShootPID;
    private SimpleMotorFeedforward m_botShootFF;
    private ProfiledPIDController m_botShootPID;

    // if this is true it lets the handoff roller motor and the shooter rollers know
    // to start spinning
    // private boolean hasPiece = false;

    Aimlock m_aim;

    /**
     * constraints in degrees
     */

    ParabolicController m_aimPC;

    BellController m_aimBC;

    /**
     * desired note speed in Meters Per Second. set kShooterSpeed to this after
     * testing
     */
    double shootSpeed;
    boolean shoot;

    private SlewRateLimiter m_slewRateLimiter;

    private ShuffleboardTab m_competitionTab;
    private SimpleWidget m_competitionIsShooterOnEntry;
    private SimpleWidget m_competitionThinksItShot;

    public ShooterSubsystem() {
        // init aim motor
        aimMotor = new CANSparkMax(ID.kShooterAimMotorID, MotorType.kBrushless);
        aimMotor.setIdleMode(IdleMode.kBrake); // todo change to brake after testing
        relativeEncoder = aimMotor.getEncoder();
        relativeEncoder.setPositionConversionFactor(20.0 / 23.142);

        topLimit = new DigitalInput(8);
        bottomLimit = new DigitalInput(9);

        m_aimPID = new ProfiledPIDController(0.5, 0.0, 0.04, new Constraints(15, 90)); // 0.19
        m_aimFF = new ArmFeedforward(0.09, 0.1, 0.625); // 0.225

        // m_aimPID = new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(15, 90));
        // m_aimFF = new ArmFeedforward(0.0, 0.0, 0.0);

        // init shooter motors
        topShooterMotor = new CANSparkMax(ID.kTopShooterMotorID, MotorType.kBrushless);
        topShooterMotor.setIdleMode(IdleMode.kCoast);
        topShooterMotor.setInverted(true);
        bottomShooterMotor = new CANSparkMax(ID.kBottomShooterMotorID, MotorType.kBrushless);
        bottomShooterMotor.setIdleMode(IdleMode.kCoast);
        bottomShooterMotor.setInverted(true);
        // bottomShooterMotor.follow(topShooterMotor, false);
        m_botShootPID = new ProfiledPIDController(0.0, 0, 0.0, new Constraints(10, 10));
        m_botShootFF = new SimpleMotorFeedforward(0.0, 0.4);

        m_topShootPID = new ProfiledPIDController(0.0, 0, 0.0, new Constraints(10, 10));
        m_topShootFF = new SimpleMotorFeedforward(0.0, 0.52);

        // get shooter speed
        shootSpeed = Shooter.kShooterSpeed;
        shoot = false;
        m_slewRateLimiter = new SlewRateLimiter(20);

        m_competitionTab = Shuffleboard.getTab("Competition Tab");
        m_competitionThinksItShot = m_competitionTab.add("Thinks It Shot", false).withSize(2, 1).withPosition(7, 2);
        m_competitionIsShooterOnEntry = m_competitionTab.add("Is Shooter On", false).withSize(2, 1).withPosition(7, 3);
    }

    public CANSparkMax getAimMotor() {
        return aimMotor;
    }

    /**
     * sets the aim motor to break mode. call this whenever the robot is enabled
     * (autoinit, teleopinit) so the hood stays where it is even while not being
     * powered
     */
    public void setAimToBreakMode() {
        aimMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * sets the aim motor to coast mode. call this on disabledinit so the hood
     * coasts down to the starting pose
     */
    public void setAimToCoastMode() {
        aimMotor.setIdleMode(IdleMode.kCoast);
    }

    /**
     * tell the hood where it is
     */
    public void setRelativeShooterEncoder(double set) {
        relativeEncoder.setPosition(set);
    }

    /**
     * @return the angle the shooter is at in degrees
     */
    public double getShooterAngleRelative() {
        return relativeEncoder.getPosition();
    }

    /**
     * shoot the piece
     */
    public void shoot() {
        shoot = true;
        System.out.println("shoot = " + shoot);
    }

    /**
     * stop trying to shoot the piece
     */
    public void noShoot() {
        shoot = false;
        System.out.println("shoot = " + shoot);
    }

    /**
     * used to set the aimlock of this class non-statically
     * 
     * @param m_aim
     */
    public void setAim(Aimlock m_aim) {
        this.m_aim = m_aim;
    }

    /**
     * the aimlock this class is using
     * 
     * @return
     */
    public Aimlock getAim() {
        return this.m_aim;
    }

    /**
     * @return Shoot motor RPM (rotations per minute)
     */
    public double getTopShootMotorRPM() {
        return topShooterMotor.getEncoder().getVelocity();
    }

    /**
     * @return Shoot motor RPS (rotations per second)
     */
    public double getTopShootMotorRPS() {
        return topShooterMotor.getEncoder().getVelocity() / 60;
    }

    /**
     * @return how fast a note would come out of the shooter in meters per second
     */
    public double getTopShooterMPS() {
        return getTopShootMotorRPS() * Math.PI * Units.inchesToMeters(4);
    }

    /**
     * @return Shoot motor RPM (rotations per minute)
     */
    public double getBottomShootMotorRPM() {
        return bottomShooterMotor.getEncoder().getVelocity();
    }

    /**
     * @return Shoot motor RPS (rotations per second)
     */
    public double getBottomShootMotorRPS() {
        return bottomShooterMotor.getEncoder().getVelocity() / 60;
    }

    /**
     * @return how fast a note would come out of the shooter in meters per second
     */
    public double getBottomShooterMPS() {
        return getBottomShootMotorRPS() * Math.PI * Units.inchesToMeters(4);
    }

    /**
     * drive the hood up and down manually
     * 
     * @param power voltage to supply to the aim motor
     */
    public void driveHoodManually(double power) {
        if (power > 0 && getTopLimit()) {
            aimMotor.setVoltage(0);
        } else {
            if (power < 0 && getBottomLimit()) {
                aimMotor.setVoltage(0);
            } else {
                aimMotor.setVoltage(power);
            }
        }
    }

    /**
     * run the shooter with the required velocities to consistently score in the
     * speaker
     */
    public void runShooterSpeakMode() {
        topShooterMotor.setVoltage(
                m_topShootFF.calculate(shootSpeed) + m_botShootPID.calculate(getTopShooterMPS(), shootSpeed));
        bottomShooterMotor.setVoltage(
                m_topShootFF.calculate(shootSpeed)
                        + m_botShootPID.calculate(getBottomShooterMPS(), shootSpeed));
    }

    /**
     * run the shooter with the required velocities to consistently score in the amp
     */
    public void runShooterAmpMode() {
        topShooterMotor.setVoltage(
                m_topShootFF.calculate(Shooter.kTopAmpShooterSpeed)
                        + m_topShootPID.calculate(getTopShooterMPS(), Shooter.kTopAmpShooterSpeed));
        bottomShooterMotor.setVoltage(
                m_botShootFF.calculate(Shooter.kBottomAmpShooterSpeed)
                        + m_botShootPID.calculate(getBottomShooterMPS(), Shooter.kBottomAmpShooterSpeed));
    }

    /**
     * run the shooter with the required velocities to consistently score in the amp
     */
    public void runShooterPassMode() {
        topShooterMotor.setVoltage(
                m_topShootFF.calculate(Shooter.kPassingShooterSpeed)
                        + m_topShootPID.calculate(getTopShooterMPS(), Shooter.kPassingShooterSpeed));
        bottomShooterMotor.setVoltage(
                m_botShootFF.calculate(Shooter.kPassingShooterSpeed)
                        + m_botShootPID.calculate(getBottomShooterMPS(), Shooter.kPassingShooterSpeed));
    }

    /**
     * set motors spinning at their desired rpms
     */
    public void spoolMotors() {
        switch (Aimlock.getDoState()) {
            case SPEAKER:
                if (Flags.pieceState.equals(Flags.subsystemsStates.loadedPiece)
                        || DriverStation.isAutonomousEnabled()) {
                    runShooterSpeakMode();
                } else {
                    topShooterMotor.setVoltage(0);
                    bottomShooterMotor.setVoltage(0);
                }
                break;
            case AMP:
                if (Flags.pieceState.equals(Flags.subsystemsStates.loadedPiece)
                        || DriverStation.isAutonomousEnabled()) {
                    runShooterAmpMode();
                } else {
                    topShooterMotor.setVoltage(0);
                    bottomShooterMotor.setVoltage(0);
                }
                break;
            case PASS:
                if (Flags.pieceState.equals(Flags.subsystemsStates.loadedPiece)
                        || DriverStation.isAutonomousEnabled()) {
                    runShooterPassMode();
                } else {
                    topShooterMotor.setVoltage(0);
                    bottomShooterMotor.setVoltage(0);
                }
                break;
            default:
                topShooterMotor.setVoltage(0);
                bottomShooterMotor.setVoltage(0);
                break;
        }

    }

    // time, velo, accel
    double previousBottomMotorData[] = { Timer.getFPGATimestamp(), 0, 0 };

    /**
     * @return returns acceleration of bottom motor
     */
    public void updateBottomShooterAcceleration() {
        double[] now = { Timer.getFPGATimestamp(), getBottomShooterMPS(), 0 };
        if (now[1] != previousBottomMotorData[1]) {
            double accel = (now[1] - previousBottomMotorData[1]) / (now[0] -
                    previousBottomMotorData[0]);
            now[2] = m_slewRateLimiter.calculate(accel);
            previousBottomMotorData = now;
        }
    }

    /**
     * @return returns if we have shot the note
     */
    public boolean hasShotNote() {
        if (((Aimlock.getDoState().equals(DoState.SPEAKER) && shoot && (previousBottomMotorData[2] < -1))
                || (Aimlock.getDoState().equals(DoState.AMP) && shoot && (previousBottomMotorData[2] < -0.5)))
                && !DriverStation.isAutonomousEnabled()) {
            return true;
        } else
            return false;
    }

    /**
     * @return returns if we have shot the note
     */
    public boolean hasShotNoteInAuto() {
        if (((Aimlock.getDoState().equals(DoState.SPEAKER) && shoot && (previousBottomMotorData[2] < -1))
                || (Aimlock.getDoState().equals(DoState.AMP) && shoot && (previousBottomMotorData[2] < -0.5)))
                && DriverStation.isAutonomousEnabled()) {
            return true;
        } else
            return false;
    }

    /**
     * aim the shooter using PID and FeedForward. pass the aimbot outputs into this
     * 
     * @param targetAngle
     */
    public void aimShooter(double targetAngle) {
        double volts = 0;
        // if (Aimlock.hasTarget()) {
        // if we see the LL target, aim at it
        volts = m_aimPID.calculate(
                getShooterAngleRelative(),
                targetAngle)
                + m_aimFF.calculate(targetAngle, targetAngle - getShooterAngleRelative());
        // volts = m_aimPC.calc(targetAngle - getShooterAngleRelative()); //todo test

        // volts = m_aimBC.calc(targetAngle - getShooterAngleRelative()); //todo test
        if (Aimlock.getDoState().equals(DoState.AMP)) {
            // if we dont see the with the limelight, go to the upper limit slowly
            volts = 3;
        }

        // check if hitting limits, if hitting limit, let the motor still drive the
        // other direction.
        if (volts > 0 && getTopLimit()) {
            volts = 0;
        } else if (volts < 0 && getBottomLimit()) {
            volts = 0;
        }

        // cap the voltage
        volts = MathUtil.clamp(volts, -5.25, 5.25);

        aimMotor.setVoltage(volts);
    }

    /**
     * @return whether or not the top limit has been triggered or not
     */
    public boolean getTopLimit() {
        return !topLimit.get();
    }

    /**
     * @return
     */
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
        updateBottomShooterAcceleration();
        // aimShooter(m_aim.getShooterTargetAngle());
        spoolMotors();
        SmartDashboard.putNumber("Shooter angle (real)", getShooterAngleRelative());
        SmartDashboard.putBoolean("top limit", getTopLimit());
        SmartDashboard.putBoolean("bottom limit", getBottomLimit());
        SmartDashboard.putNumber("top shoot", getTopShooterMPS());
        SmartDashboard.putNumber("bot shoot", getBottomShooterMPS());
        // SmartDashboard.putNumber("des shoot speed", getDesiredShooterSpeed());
        // SmartDashboard.putNumber("volts to hood", aimMotor.getAppliedOutput());
        SmartDashboard.putNumber("target shooter angle",
                m_aim.getShooterTargetAngle());
        SmartDashboard.putNumber("Bottom Shooter Accel", previousBottomMotorData[2]);
        SmartDashboard.putBoolean("has shot?", hasShotNote());
        SmartDashboard.putBoolean("shooting?", shoot);
        SmartDashboard.putBoolean("is loaded?", Flags.pieceState.equals(Flags.subsystemsStates.loadedPiece));
        // SmartDashboard.putBoolean("speaker mode?",
        // Aimlock.getDoState().equals(Aimlock.DoState.SPEAKER));
        // SmartDashboard.putBoolean("top decel bool", previousTopMotorData[2] < -3);
        // SmartDashboard.putBoolean("down decel bool", previousBottomMotorData[2] <
        // -3);
        // SmartDashboard.putNumber("Target angle", (m_aim.getShooterTargetAngle()));
        // SmartDashboard.putNumber("vert angle speaker",
        // Math.toDegrees(m_aim.getVerticalAngleToSpeaker()));
        // SmartDashboard.putNumber("TY", LimelightHelpers.getTY("limelight-a"));
        // SmartDashboard.putNumber("dist speaker", m_aim.getDistToSpeaker());

        m_competitionIsShooterOnEntry.getEntry().setBoolean(hasShotNote());
        m_competitionIsShooterOnEntry.getEntry().setBoolean(shoot);
    }
}
