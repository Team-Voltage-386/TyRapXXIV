// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ID;
import frc.robot.Constants.Offsets;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveTrain;
import frc.robot.Utils.Aimlock;
import frc.robot.Utils.Aimlock.DoState;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
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
    private final ShuffleboardTab m_driveTab = Shuffleboard.getTab("drive subsystem");
    private final SimpleWidget m_fieldRelativeWidget = m_driveTab.add("drive field relative", fieldRelative);

    private Aimlock m_aim;

    /**
     * The order that you initialize these is important! Later uses of functions
     * like toSwerveModuleStates will return the same order that these are provided.
     * See
     * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
     */
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
    private boolean lockTargetInAuto = false;

    private ExecutorService executorService = Executors.newFixedThreadPool(4);

    public Drivetrain(Pigeon2 gyro, CameraSubsystem camera) {
        // Zero at beginning of match. Zero = whatever direction the robot (more
        // specifically the gyro) is facing
        this.m_gyro = gyro;
        this.resetGyro();
        this.m_camera = camera;

        m_odometry = new SwerveDriveOdometry(
                m_kinematics,
                getGyroYawRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                });

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getRoboPose2d, // Robot pose supplier
                this::resetOdo, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveInAuto, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(4, 0.1, 0.0), // Translation PID constants p used to be 7
                        new PIDConstants(5, 0.01, 0.0), // Rotation PID constants
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
        lockTargetInAuto = lock;
        System.out.println("toggled auto lock to " + lockTargetInAuto);
    }

    public Pigeon2 getGyro() {
        return m_gyro;
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
        if (pose != null) {
            m_odometry.resetPosition(getGyroYawRotation2d(), getModulePositions(), pose);
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_chassisSpeeds;
    }

    public boolean getFieldRelative() {
        return fieldRelative;
    }

    public void setFieldRelative(boolean isFieldRelative) {
        fieldRelative = isFieldRelative;
        m_fieldRelativeWidget.getEntry().setBoolean(fieldRelative);
    }

    public Command setFieldRelativeCommand(boolean isFieldRelative) {
        return runOnce(() -> {
            this.setFieldRelative(isFieldRelative);
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
        CountDownLatch latch = new CountDownLatch(4);
        executorService.execute(() -> {
            m_frontLeft.setDesiredState(swerveModuleStates[0]);
            latch.countDown();
        });
        executorService.execute(() -> {
            m_frontRight.setDesiredState(swerveModuleStates[1]);
            latch.countDown();
        });
        executorService.execute(() -> {
            m_backLeft.setDesiredState(swerveModuleStates[2]);
            latch.countDown();
        });
        executorService.execute(() -> {
            m_backRight.setDesiredState(swerveModuleStates[3]);
            latch.countDown();
        });
        try {
            latch.await();
        } catch (InterruptedException e) {
            // Pass
        }

        // m_frontLeft.setDesiredState(swerveModuleStates[0]);
        // m_frontRight.setDesiredState(swerveModuleStates[1]);
        // m_backLeft.setDesiredState(swerveModuleStates[2]);
        // m_backRight.setDesiredState(swerveModuleStates[3]);

        SmartDashboard.putNumber("desired X speed", xSpeed);
        SmartDashboard.putNumber("desired Y speed", ySpeed);
        // this.layout.setDesiredRotSpeed(Math.toDegrees(rotSpeed));
    }

    public void lockTarget(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative, boolean hardLocked) {
        SwerveModuleState[] swerveModuleStates; // MAKE SURE swervestates can be init like this with this kinda array
        if (Aimlock.hasTarget() || (Aimlock.hasNoteTarget() && Aimlock.getNoteVision())
                || Aimlock.getDoState().equals(DoState.AMP)) {
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

        // this.layout.setDesiredXSpeed(xSpeed);
        // this.layout.setDesiredYSpeed(ySpeed);
        // this.layout.setDesiredRotSpeed(Math.toDegrees(rotSpeed));
    }

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
                                        : chassisSpeeds.omegaRadiansPerSecond, // test this
                                getGyroYawRotation2d())
                        : chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxPossibleSpeed);

        // this.layout.setDesiredXSpeed(chassisSpeeds.vxMetersPerSecond);
        // this.layout.setDesiredYSpeed(chassisSpeeds.vyMetersPerSecond);
        // this.layout.setDesiredRotSpeed(Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond));

        // passing back the math from kinematics to the swerves themselves.
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public Pose2d getRoboPose2d() {
        return m_odometry.getPoseMeters();
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
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Chassis Angle",
                getRoboPose2d().getRotation().getDegrees());
        SmartDashboard.putNumber("Desired Angle",
                Math.toDegrees(m_aim.getSpeakerAimTargetAngle()));
        SmartDashboard.putNumber("Ang to Speak", m_aim.getAngleToSpeaker());
        // SmartDashboard.putNumber("X speed", getChassisSpeeds().vxMetersPerSecond);
        // SmartDashboard.putNumber("Y speed", getChassisSpeeds().vyMetersPerSecond);
        // SmartDashboard.putNumber("X pos", getRoboPose2d().getX());
        // SmartDashboard.putNumber("Y pos", getRoboPose2d().getY());

        if (!DriverStation.isAutonomousEnabled()) {
            resetOdo(m_camera.resetOdoLimelight());
        }
        updateOdometry();
    }
}
