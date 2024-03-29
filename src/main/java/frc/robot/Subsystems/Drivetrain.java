// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ID;
import frc.robot.Constants.Offsets;
import frc.robot.Utils.LimelightHelpers;
import frc.robot.Utils.Aimlock.DoState;
import frc.robot.Utils.Aimlock;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveTrain;
import frc.robot.Constants.PipeLineID;
import frc.robot.ShuffleboardLayouts.DrivetrainLayout;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
    public static enum DirectionOption {
        FORWARD,
        BACKWARD
    }

    public static final double kMaxPossibleSpeed = 5; // meters per second
    public static final double kMaxAngularSpeed = 3 * Math.PI; // per second

    private final Translation2d m_frontLeftLocation = new Translation2d(
            DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXForward,
            DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYLeft);
    private final Translation2d m_frontRightLocation = new Translation2d(
            DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXForward,
            DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYRight);
    private final Translation2d m_backLeftLocation = new Translation2d(
            DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXBackward,
            DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYLeft);
    private final Translation2d m_backRightLocation = new Translation2d(
            DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXBackward,
            DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYRight);

    private final SwerveModule m_frontLeft = new SwerveModule("FrontLeft",
            ID.kFrontLeftDrive,
            ID.kFrontLeftTurn,
            ID.kFrontLeftCANCoder,
            Offsets.kFrontLeftOffset,
            DriveTrain.turnPID,
            DriveTrain.drivePID,
            DriveTrain.turnFeedForward,
            DriveTrain.driveFeedForward);
    private final SwerveModule m_frontRight = new SwerveModule("FrontRight",
            ID.kFrontRightDrive,
            ID.kFrontRightTurn,
            ID.kFrontRightCANCoder,
            Offsets.kFrontRightOffset,
            DriveTrain.turnPID,
            DriveTrain.drivePID,
            DriveTrain.turnFeedForward,
            DriveTrain.driveFeedForward);
    private final SwerveModule m_backLeft = new SwerveModule("BackLeft",
            ID.kBackLeftDrive,
            ID.kBackLeftTurn,
            ID.kBackLeftCANCoder,
            Offsets.kBackLeftOffset,
            DriveTrain.turnPID,
            DriveTrain.drivePID,
            DriveTrain.turnFeedForward,
            DriveTrain.driveFeedForward);
    private final SwerveModule m_backRight = new SwerveModule("BackRight",
            ID.kBackRightDrive,
            ID.kBackRightTurn,
            ID.kBackRightCANCoder,
            Offsets.kBackRightOffset,
            DriveTrain.turnPID,
            DriveTrain.drivePID,
            DriveTrain.turnFeedForward,
            DriveTrain.driveFeedForward);

    private final Pigeon2 m_gyro;
    private final CameraSubsystem m_camera;

    private boolean fieldRelative = true;
    private DirectionOption m_forwardDirection = DirectionOption.FORWARD;
    private final ShuffleboardTab m_driveTab = Shuffleboard.getTab("drive subsystem");
    private final SimpleWidget m_fieldRelativeWidget = m_driveTab.add("drive field relative", fieldRelative);

    private DrivetrainLayout layout = new DrivetrainLayout();

    /**
     * The order that you initialize these is important! Later uses of functions
     * like toSwerveModuleStates will return the same order that these are provided.
     * See
     * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
     */
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry;
    private Aimlock m_aim;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
    private Pose2d robotFieldPosition;
    private boolean lockTargetInAuto = false;

    public Drivetrain(Pigeon2 m_gyro, CameraSubsystem m_camera) {
        // Zero at beginning of match. Zero = whatever direction the robot (more
        // specifically the gyro) is facing
        this.m_gyro = m_gyro;
        this.resetGyro();

        this.m_camera = m_camera;

        m_odometry = new SwerveDriveOdometry(
                m_kinematics,
                getGyroYawRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                },
                new Pose2d(
                        new Translation2d(2.0, 6.0), Rotation2d.fromDegrees(0.0)));

        robotFieldPosition = getRoboPose2d();

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getRoboPose2d, // Robot pose supplier
                this::resetOdo, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveInAuto, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(7.4, 0.0, 0.0), // Translation PID constants p used to be 7
                        new PIDConstants(5.4, 0.0, 0.0), // Rotation PID constants
                        kMaxPossibleSpeed, // Max module speed, in m/s
                        DriveTrain.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to
                                                     // furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public void setAim(Aimlock m_aim) {
        this.m_aim = m_aim;
    }

    public void setLockTargetInAuto(boolean lock) {
        System.out.println("toggled auto lock to" + lockTargetInAuto);
        lockTargetInAuto = lock;
    }

    /**
     * Resets Orientation of the robot
     */
    public void resetGyro() {
        m_gyro.setYaw(0);
    }

    /**
     * Resets robot position on the field
     */
    public void resetOdo() {
        m_odometry.resetPosition(getGyroYawRotation2d(), getModulePositions(),
                new Pose2d(new Translation2d(3, 7), new Rotation2d()));
    }

    /**
     * Resets Odometry using a specific Pose2d
     * 
     * @param pose
     */
    public void resetOdo(Pose2d pose) {
        // System.out.println(getGyroYawRotation2d().getDegrees());
        // System.out.println(pose.getX() + " " + pose.getY() + " " +
        // pose.getRotation().getDegrees());
        if (pose != null)
            m_odometry.resetPosition(getGyroYawRotation2d(), getModulePositions(), pose);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_chassisSpeeds;
    }

    public DirectionOption getDirectionOption() {
        return this.m_forwardDirection;
    }

    public Command setDirectionOptionCommand(DirectionOption option) {
        return runOnce(() -> {
            this.m_forwardDirection = option;
        });
    }

    public boolean getFieldRelative() {
        return fieldRelative;
    }

    public Command toggleFieldRelativeCommand() {
        return runOnce(() -> {
            fieldRelative = !fieldRelative;
            m_fieldRelativeWidget.getEntry().setBoolean(fieldRelative);
        });
    }

    /**
     * Module positions in the form of SwerveModulePositions (Module orientation and
     * the distance the wheel has travelled across the ground)
     * 
     * @return SwerveModulePosition[]
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
        };
    }

    /**
     * Sends swerve info to smart dashboard
     */
    public void print() {
        m_frontLeft.print();
        m_frontRight.print();
        m_backLeft.print();
        m_backRight.print();
    }

    /**
     * Get the yaw of gyro in Rotation2d form
     * 
     * @return chasis angle in Rotation2d
     */
    public Rotation2d getGyroYawRotation2d() {
        // return Rotation2d.fromDegrees(MathUtil.inputModulus(m_gyro.getYaw(), -180,
        // 180));
        return Rotation2d.fromDegrees(m_gyro.getYaw().getValue());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rotSpeed      Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed) {
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getGyroYawRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxPossibleSpeed);

        // passing back the math from kinematics to the swerves themselves.
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);

        this.layout.setDesiredXSpeed(xSpeed);
        this.layout.setDesiredYSpeed(ySpeed);
        this.layout.setDesiredRotSpeed(Math.toDegrees(rotSpeed));
    }

    /**
     * Pathplanner uses this method in order to interface with our Drivetrain.
     * 
     * @param chassisSpeeds Robot relative ChassisSpeeds of the robot containing X,
     *                      Y, and Rotational Velocities.
     */
    public void driveInAuto(ChassisSpeeds chassisSpeeds) {
        // SwerveModuleState[] swerveModuleStates =
        // m_kinematics.toSwerveModuleStates(chassisSpeeds);
        // comment: if speakermode, use speakeraim, if not speakermode, use piece aim.
        // if using piece aim and dont see a piece, follow normal path.
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                lockTargetInAuto
                        ? ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds.vxMetersPerSecond,
                                chassisSpeeds.vyMetersPerSecond,
                                Aimlock.hasTarget() ? m_aim.getRotationSpeedForTarget()
                                        : chassisSpeeds.omegaRadiansPerSecond, // comment
                                getGyroYawRotation2d())
                        : chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxPossibleSpeed);

        this.layout.setDesiredXSpeed(chassisSpeeds.vxMetersPerSecond);
        this.layout.setDesiredYSpeed(chassisSpeeds.vyMetersPerSecond);
        this.layout.setDesiredRotSpeed(Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond));

        // passing back the math from kinematics to the swerves themselves.
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * What this code essentially does is when you're holding down the left button
     * (in robot), just drive like normal until you see a game piece.
     * once you see it, its locked. from this point, you may drive around as normal,
     * but the aimlock will handle the robot's orientation.
     * If you then press the left trigger (in robot), the robot will "hard lock"
     * onto the piece, and you will only be able
     * to go forwards and backwards (robot oriented), in order to perfectly pick up
     * the piece.
     * With proper tuning it will be able to lock, and stay locked onto the
     * tag/piece no matter how fast we are driving.
     */
    public void lockPiece(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative, boolean hardLocked) {
        SwerveModuleState[] swerveModuleStates; // MAKE SURE swervestates can be init like this with this kinda array
        if (Aimlock.hasTarget()) {
            rotSpeed = m_aim.getRotationSpeedForTarget();
            if (hardLocked) {
                swerveModuleStates = m_kinematics.toSwerveModuleStates(
                        new ChassisSpeeds(xSpeed, 0, rotSpeed));
            } else {
                swerveModuleStates = m_kinematics.toSwerveModuleStates(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                xSpeed, ySpeed, rotSpeed, getGyroYawRotation2d()));
            }
        } else {
            if (hardLocked) {
                swerveModuleStates = m_kinematics.toSwerveModuleStates(
                        new ChassisSpeeds(xSpeed, 0, rotSpeed));
            } else {
                swerveModuleStates = m_kinematics.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed,
                                        getGyroYawRotation2d())
                                : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
            }
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxPossibleSpeed);

        // passing back the math from kinematics to the swerves themselves.
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);

        this.layout.setDesiredXSpeed(xSpeed);
        this.layout.setDesiredYSpeed(ySpeed);
        this.layout.setDesiredRotSpeed(Math.toDegrees(rotSpeed));
    }

    public Pose2d getRoboPose2d() {
        return m_odometry.getPoseMeters();
    }

    /**
     * THIS IS IN DEGREES
     * Triangulates position of robot knowing the distance between two april tags
     * seen by the camera.
     * 
     * @param length
     * @param angle1
     * @param angle2
     * @return
     */
    public Pose2d calcRoboPose2dWithVision(double length, double angle1, double angle2) {
        double L = length; // dist between the two april tags
        double a1 = angle1; // angle (from the camera) of the close april tag (a1) and the far april tag
                            // (a2)
        double a2 = angle2;
        double gyroOffset = 0;
        double roboAngle = -m_gyro.getYaw().getValue() + gyroOffset; // angle of the robot (0 degrees = facing the drivers)

        double X = (L * Math.sin(Math.toRadians(90 + roboAngle + a2)) * Math.sin(Math.toRadians(90 - roboAngle - a1)))
                / Math.sin(Math.toRadians(Math.abs(a2 - a1)));

        double Y = (L * Math.sin(Math.toRadians(90 + roboAngle + a2)) * Math.cos(Math.toRadians(90 - roboAngle - a1)))
                / Math.sin(Math.toRadians(Math.abs(a2 - a1)));

        return new Pose2d(X, Y, getGyroYawRotation2d());
    }

    public void stopDriving() {
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        0,
                        0,
                        0,
                        getGyroYawRotation2d()));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxPossibleSpeed);

        // passing back the math from kinematics to the swerves themselves.
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {

        m_odometry.update(
                getGyroYawRotation2d(),
                getModulePositions());

        // getting velocity vectors from each module
        SwerveModuleState frontLeftState = m_frontLeft.getState();
        SwerveModuleState frontRightState = m_frontRight.getState();
        SwerveModuleState backLeftState = m_backLeft.getState();
        SwerveModuleState backRightState = m_backRight.getState();

        // Converting module speeds to chassis speeds
        m_chassisSpeeds = m_kinematics.toChassisSpeeds(
                frontLeftState, frontRightState, backLeftState, backRightState);
        robotFieldPosition = getRoboPose2d();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Target", Math.toDegrees(m_aim.getSpeakerAimTargetAngle()));
        SmartDashboard.putNumber("rot", getRoboPose2d().getRotation().getDegrees());
        resetOdo(m_camera.resetOdoLimelight());
        updateOdometry();

        this.layout.setLimelightFRTargetAngle(getRoboPose2d().getRotation().getDegrees() - LimelightHelpers.getTX(""));
        this.layout.setActualAngleToSpeaker(m_aim.getAngleToSpeaker());
        this.layout.setDesiredAngleToSpeaker(Math.toDegrees(m_aim.getSpeakerAimTargetAngle()));
        this.layout.setLimelightRRTargetAngle(m_aim.getLLAngleToTarget());
        this.layout.setxPos(m_odometry.getPoseMeters().getX());
        this.layout.setyPos(m_odometry.getPoseMeters().getY());
        this.layout.setRot(m_odometry.getPoseMeters().getRotation().getDegrees());
    }
}
