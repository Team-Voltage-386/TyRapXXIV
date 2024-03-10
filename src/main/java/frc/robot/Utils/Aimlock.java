package frc.robot.Utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PipeLineID;
import frc.robot.Constants.Shooter;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.ShooterSubsystem;

public class Aimlock {
    Drivetrain m_swerve;
    ShooterSubsystem m_shooter;
    private static int pipeline;
    static boolean noteVision = false;

    /**
     * what state the robot is in
     */
    public static enum DoState {
        NOTE,
        SPEAKER,
        AMP,
        SOURCE,
        ENDGAME
    }

    private static DoState doState = DoState.SPEAKER;

    public Aimlock(Drivetrain m_swerve, ShooterSubsystem m_shooter) {
        this.m_swerve = m_swerve;
        this.m_shooter = m_shooter;
        setPipeline(PipeLineID.kSpeakerID);
        LimelightHelpers.setPipelineIndex("limelight-c", 0);
    }

    // PID/FF for chassis rotation speed
    private SimpleMotorFeedforward aimFF = new SimpleMotorFeedforward(0.0, 12);
    private ProfiledPIDController aimPID = new ProfiledPIDController(5, 0.35, 0.4,
            new Constraints(Math.toRadians(180), Math.toRadians(180)));

    private SimpleMotorFeedforward RRaimFF = new SimpleMotorFeedforward(0.0, 0.0);
    private ProfiledPIDController RRaimPID = new ProfiledPIDController(3, 0.0, 0.1,
            new Constraints(Math.toRadians(180), Math.toRadians(180)));

    private static final String limelightName = "limelight-b";
    private double limelightHeight = Units.inchesToMeters(26);
    private double targetTagHeight = Units.inchesToMeters(51.88);
    private double speakerHeight = Units.inchesToMeters(79.5);

    /**
     * select the pipeline you want to use
     * 
     * @param pipeLineIndex
     */
    public static void setPipeline(int pipeLineIndex) {
        LimelightHelpers.setPipelineIndex(limelightName, pipeLineIndex);
        pipeline = pipeLineIndex;
    }

    public static void setNoteVision(boolean noteVision) {
        Aimlock.noteVision = noteVision;
    }

    public static boolean getNoteVision() {
        return noteVision;
    }

    /**
     * returns which pipeline is currently being used. (default is speaker)
     * 
     * @return
     */
    public static int getPipeLine() {
        return pipeline;
    }

    /**
     * @return what state the robot is in
     */
    public static DoState getDoState() {
        return doState;
    }

    /**
     * set what state the robot is in
     * 
     * @param state
     */
    public static void setDoState(DoState state) {
        doState = state;
        switch (doState) {
            case NOTE:
                setPipeline(PipeLineID.kNoteID);
                break;
            case SPEAKER:
                setPipeline(PipeLineID.kSpeakerID);
                break;
            case AMP:
                setPipeline(PipeLineID.kAmpID);
                break;
            case SOURCE:
                setPipeline(PipeLineID.kSourceID);
                break;
            case ENDGAME:
                break;
            default:
                setPipeline(PipeLineID.kNoteID);
                break;
        }
    }

    /**
     * @return if the limelight has a valid target
     */
    public static boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    public static boolean hasNoteTarget() {
        return LimelightHelpers.getTV("limelight-c");
    }

    /**
     * get field relative angle to speaker
     */
    public double getAngleToSpeaker() {
        return getLLFRAngleToTarget();
    }

    /**
     * get field relative target angle for swerve to aim at speaker
     * 
     * @return radians
     */
    public double getSpeakerAimTargetAngle() {
        double Vy = m_swerve.getChassisSpeeds().vyMetersPerSecond
                + getShooterSpeedwDrag() * Math.sin(Math.toRadians(getAngleToSpeaker() + 180));
        double Vx = m_swerve.getChassisSpeeds().vxMetersPerSecond
                + getShooterSpeedwDrag() * Math.cos(Math.toRadians(getAngleToSpeaker() + 180));
        return 2 * Math.toRadians(getAngleToSpeaker()) - Math.atan(Vy / Vx);
        // return Math.atan(Vy / Vx); // dont know why this suddenly isnt inverted also
        // (found out i think tho)
        // consider using gyro and not swerve odo
    }

    public double getShooterSpeedwDrag() {
        SmartDashboard.putNumber("drag shoot spd",
                Shooter.kShooterSpeed - Shooter.kDragCoefficient * Math.sqrt(getDistToSpeaker()));
        return Shooter.kShooterSpeed - Shooter.kDragCoefficient * Math.sqrt(getDistToSpeaker());
    }

    /**
     * robot relative aimbot for non-speaker things
     * 
     * @return degrees to the target, right is +, left is -
     */
    public double getLLAngleToTarget() {
        if (hasTarget()) {
            return LimelightHelpers.getTX(limelightName);
        } else
            return 0;
    }

    public double getNoteLLAngleToTarget() {
        if (hasNoteTarget()) {
            return LimelightHelpers.getTX("limelight-c");
        } else
            return 0;
    }

    /**
     * returns field relative angle thats compliant with WPI cause WPI has rotation
     * backwards
     * 
     * @return degrees to the target, right is -, left is +
     */
    public double getLLFRAngleToTarget() {
        double angle = getGyroYaw() - LimelightHelpers.getTX(limelightName);
        if (hasTarget())
            return angle;
        else
            return getGyroYaw();
    }

    /**
     * @return returns rotation like wpi likes it, left is positive right is
     *         negative
     */
    public double getGyroYaw() {
        return -m_swerve.getGyroYawRotation2d().getDegrees();// m_swerve.getRoboPose2d().getRotation().getDegrees();
    }

    /**
     * calc how fast robot needs to turn to stay with target
     * 
     * @return rot speed
     */
    public double getRotationSpeedForTarget() {
        if (noteVision) {
            return -RRaimFF.calculate(Math.toRadians(getNoteLLAngleToTarget()))
                    + RRaimPID.calculate(Math.toRadians(getNoteLLAngleToTarget()));
        }
        if (!doState.equals(DoState.SPEAKER)) {
            return -RRaimFF.calculate(Math.toRadians(getLLAngleToTarget()))
                    + RRaimPID.calculate(Math.toRadians(getLLAngleToTarget()));
        } else {
            return (aimFF.calculate(getSpeakerAimTargetAngle() - Math.toRadians(getGyroYaw()))
                    + aimPID.calculate(Math.toRadians(getGyroYaw()), getSpeakerAimTargetAngle()))
                    / 3; // if you remove the /3 the earth will explode
        }
    }

    /**
     * returns pure horizontal distance to tag
     */
    public double getDistToTag() {
        return (targetTagHeight - limelightHeight)
                / Math.tan(Math.toRadians(LimelightHelpers.getTY(limelightName)));
    }

    /**
     * distance to speaker (the hypotenuse, so true distance.)
     */
    public double getDistToSpeaker() {
        return Math.hypot(getDistToTag(), speakerHeight);
    }

    /**
     * @return (radians) angle the shooter would need to be at to be pointed
     *         directly at the speaker
     */
    public double getVerticalAngleToSpeaker() {
        return Math.atan(speakerHeight / getDistToTag());
    }

    /**
     * @return the angle the shooter must be at in order to score in the speaker
     *         (accounting
     *         for robot movement and projectile drop)
     */
    public double getShooterTargetAngleSPEAKER() {
        if (!hasTarget()) {
            return 10; // go to 10 degrees if u no see
        }

        double Vy = getShooterSpeedwDrag() * Math.sin(getVerticalAngleToSpeaker()); // vertical vector of note
        double Vx = getShooterSpeedwDrag() * Math.cos(getVerticalAngleToSpeaker())
                - m_swerve.getChassisSpeeds().vxMetersPerSecond;
        // - m_swerve.getChassisSpeeds().vxMetersPerSecond; // horizontal vector
        // of
        // note
        // the angle that the shooter WILL shoot the note at if we aim directly at the
        // target.
        double perfectAngle = Math.atan(Vy / Vx);
        // accounting for drop
        // (9.80665t is the velocity of a falling object after time t has passed)
        // try (9.8t^2)/2 if this proves inconsistent. t = ground distance/ground speed
        // horizontal vector
        double noteDropMeters = ((9.80665 * Shooter.kFallingDragCoefficient
                * Math.pow((getDistToTag() / (getShooterSpeedwDrag() * Math.cos(perfectAngle))), 2))) / 2;
        double tuningFactor = 1.06; // tweak how much it adjusts
        // the angle we NEED to shoot at to hit the target, including drop. (just the
        // amount of degrees
        // of error in the other direction)
        double angle = Math.toDegrees(2 * getVerticalAngleToSpeaker() - (tuningFactor
                * perfectAngle));
        // double angle = Math.toDegrees((tuningFactor * perfectAngle));
        double angleWithDrop = angle - Math.toDegrees(
                Math.atan(((Vy / Vx) * getDistToTag() + noteDropMeters) / getDistToTag()) - perfectAngle);
        SmartDashboard.putNumber("angle w drop", angleWithDrop - 32);
        SmartDashboard.putNumber("note drop", noteDropMeters);
        // when backing away turret should move down more
        // when coming forward turret should move up more
        // 32 is the bottom of the shooter angle IRL
        // 3 is just a guestimate of from where we measure to where the note actually
        // comes out
        return MathUtil.clamp(angleWithDrop - 32.0, Shooter.kMinAngle, Shooter.kMaxAngle);
    }

    /**
     * @return the maximum angle of the shooter
     */
    public double getShooterTargetAngleAMP() {
        return Shooter.kMaxAngle;
    }

    /**
     * @return the maximum angle of the shooter
     */
    public double getShooterTargetAngleSOURCE() {
        return Shooter.kMaxAngle;
    }

    // angle the shooter needs to hit the shot. ask me (Lucas) about the math if you
    // need to cause I dont have time to comment it all rn
    /**
     * In degrees, performs math for the necessary shooter angle taking into account
     * swerve velocity.
     * 
     * @return target angle of the shooter
     */
    public double getShooterTargetAngle() {
        double angle = Shooter.kMinAngle;
        switch (doState) {
            case NOTE:
                break;
            case SPEAKER:
                angle = getShooterTargetAngleSPEAKER();
                break;
            case AMP:
                angle = getShooterTargetAngleAMP();
                break;
            case SOURCE:
                angle = getShooterTargetAngleSOURCE();
                break;
            case ENDGAME:
                angle = Shooter.kMinAngle;
                break;
            default:
                break;
        }
        return angle;
    }
}
