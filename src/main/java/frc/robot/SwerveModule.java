// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Modules;

public class SwerveModule {
    private static final int kEncoderResolution = 4096;

    private static final double kModuleMaxAngularVelocity = 8 * Math.PI;
    private static final double kModuleMaxAngularAcceleration = 20 * Math.PI; // radians per second squared this was
                                                                              // 2PI

    /**
     * The drive motor is responsible for the actual power across the ground e.g. to
     * make it move forward/backward
     */
    private final SparkMax m_driveMotor;
    private final SparkMaxConfig m_driveConfig;

    /**
     * The turning/steering motor is responsible for the orientation of the wheel
     * e.g. to make it turn
     */
    private final SparkMax m_turningMotor;
    private final SparkMaxConfig m_turningConfig;

    /**
     * Each turning motor has an encoder. Each motor was mounted by people at
     * seemingly random offsets. We have to accommodate for these
     * offsets so that calls to getAbsolutePosition all return the same value
     * regardless of motor i.e. make them consider the same reference point as "0"
     */
    public final double m_encoderOffset;

    /**
     * This does not control the motor. This just records the orientation of the
     * module.
     * IMPORTANT NOTE: Ensure that the encoder and motor both agree which
     * direction is positive.
     * We faced an issue where the motor oscillated inexplicably and we traced it
     * back to the motor needing to be inverted to match with the encoder.
     */
    private final CANcoder m_turningEncoder;

    /**
     * Identification for what motor this is
     */
    private final String m_swerveModuleName;

    private final PIDController m_drivePIDController;

    private final ProfiledPIDController m_turningPIDController;

    private double m_turningLastSpeed = 0;
    private double m_turningLastTime = Timer.getFPGATimestamp();
    private double m_turningLastPosition = 0.0;

    // Example code came with these feed forward pieces which we haven't yet added
    // as they are meant for tuning and are not required
    // We leave them here in case you'd like to reference them
    private final SimpleMotorFeedforward m_driveFeedforward;
    private final SimpleMotorFeedforward m_turnFeedforward;

    private SwerveModuleState m_desiredState = new SwerveModuleState();

    // private GenericEntry encoderOffset;
    // private GenericEntry absEncoderOffset;
    // private GenericEntry desiredTurningSpeed;
    // private GenericEntry actualTurningSpeed;
    // private GenericEntry desiredDriveSpeed;
    // private GenericEntry actualDriveSpeed;

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
     * and turning encoder.
     *
     * @param driveMotorChannel      PWM output for the drive motor.
     * @param turningMotorChannel    PWM output for the turning motor.
     * @param driveEncoderChannelA   DIO input for the drive encoder channel A
     * @param driveEncoderChannelB   DIO input for the drive encoder channel B
     * @param turningEncoderChannelA DIO input for the turning encoder channel A
     * @param turningEncoderChannelB DIO input for the turning encoder channel B
     */
    public SwerveModule(
            String swerveModuleID,
            int driveMotorChannel,
            int turningMotorChannel,
            int turningEncoderID,
            double encoderOffset,
            double[] steerPID,
            double[] drivePID,
            double[] turnFeedForward,
            double[] driveFeedForward) {

        m_drivePIDController = new PIDController(
                drivePID[0],
                drivePID[1],
                drivePID[2]);

        m_turningPIDController = new ProfiledPIDController(
                steerPID[0],
                steerPID[1],
                steerPID[2],
                new TrapezoidProfile.Constraints(
                        kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

        m_swerveModuleName = swerveModuleID;

        /*
         * Set up the drive motor
         */
        m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
        m_driveConfig = new SparkMaxConfig();
        m_driveMotor.getEncoder().setPosition(0);
        m_driveConfig.smartCurrentLimit(40);
        m_driveConfig.alternateEncoder.positionConversionFactor(Modules.kDriveEncoderRot2Meter);
        m_driveConfig.alternateEncoder.velocityConversionFactor(Modules.kDriveEncoderRPM2MeterPerSec);
        m_driveConfig.idleMode(IdleMode.kBrake);
        m_driveConfig.inverted(false);
        m_driveMotor.configure(m_driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        /*
         * Set up the turning motor. We had to invert the turning motor so it agreed
         * with the turning encoder which direction was positive
         */
        m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);
        m_turningConfig = new SparkMaxConfig();
        m_turningConfig.inverted(false);
        m_turningConfig.smartCurrentLimit(40);
        m_turningConfig.idleMode(IdleMode.kBrake);
        m_turningMotor.configure(m_turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        /*
         * Set up and configure the turning encoder. Additional config is required to
         * ensure that units are correct
         */
        m_turningEncoder = new CANcoder(turningEncoderID);

        // Want absolute readings [-0.5, 0.5)
        MagnetSensorConfigs turningEncoderMagnetSensorConfigs = new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(0.5)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
        m_turningEncoder.getConfigurator().apply(turningEncoderMagnetSensorConfigs);
        m_turningEncoder.clearStickyFaults();

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        m_encoderOffset = encoderOffset;

        m_turnFeedforward = new SimpleMotorFeedforward(turnFeedForward[0], turnFeedForward[1]);
        m_driveFeedforward = new SimpleMotorFeedforward(driveFeedForward[0], driveFeedForward[1]);

        // this.encoderOffset = Shuffleboard.getTab(m_swerveModuleName)
        // .add("Relative Encoder Offset (Radians)", 0.0)
        // .withPosition(0, 0).withSize(2, 1).getEntry();
        // this.absEncoderOffset = Shuffleboard.getTab(m_swerveModuleName)
        // .add("Absolute Encoder Offset (Degrees)", 0.0)
        // .withPosition(0, 1).withSize(2, 1).getEntry();

        // this.desiredTurningSpeed =
        // Shuffleboard.getTab(m_swerveModuleName).add("Desired Turning Speed", 0.0)
        // .withPosition(3, 0).withSize(2, 1).getEntry();
        // this.actualTurningSpeed = Shuffleboard.getTab(m_swerveModuleName).add("Actual
        // Turning Speed", 0.0)
        // .withPosition(3, 1).withSize(2, 1).getEntry();

        // this.desiredDriveSpeed = Shuffleboard.getTab(m_swerveModuleName).add("Desired
        // Drive Speed", 0.0)
        // .withPosition(3, 3).withSize(2, 1).getEntry();
        // this.actualDriveSpeed = Shuffleboard.getTab(m_swerveModuleName).add("Actual
        // Drive Speed", 0.0)
        // .withPosition(3, 4).withSize(2, 1).getEntry();
    }

    /**
     * Returns the current Swerve Module state. This includes info about the module
     * wheel's speed per second and the angle of the module. Units are based on the
     * configuration of the motors and encoders
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                m_driveMotor.getEncoder().getVelocity(), new Rotation2d(getActualTurningPosition()));
    }

    public SwerveModuleState getDesiredState() {
        return m_desiredState;
    }

    /**
     * Returns the current position of the module. This includes info about the
     * distance measured by the wheel and the angle of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getActualDrivePosition(),
                new Rotation2d(getActualTurningPosition()));
    }

    /**
     * Returns the distance the wheel thinks it has gone across the floor. We use
     * math here instead of built-in drive conversion for ease of use
     * 
     * @return distance wheel has gone across the floor. (Circumference*rotations)
     */
    public double getActualDrivePosition() {
        return m_driveMotor.getEncoder().getPosition(); // USING CONVERSION FACTOR IN MODULE CONSTANTS
    }

    /**
     * Returns orientation of wheel, accounting for encoder offsets. 0 is when
     * aligned with forward axis of the chasis.
     * 
     * @return real orientation of wheel.
     */
    public double getActualTurningPosition() {
        double ans = (m_turningEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI) - m_encoderOffset;
        double ansMod = MathUtil.angleModulus(ans); // keep between -PI to PI
        return ansMod;
    }

    public void resetDriveError() {
        m_drivePIDController.reset();
    }

    // Controls a simple motor's position using a SimpleMotorFeedforward
    // and a ProfiledPIDController
    public void goToPosition(double goalPosition) {
        double targetVelocity = m_turningPIDController.getSetpoint().velocity;
        double targetAcceleration = (targetVelocity - this.m_turningLastSpeed)
                / (Timer.getFPGATimestamp() - this.m_turningLastTime);

        double actualVelocity = 2 * Math.PI * m_turningEncoder.getVelocity().getValueAsDouble();

        double pidVal = m_turningPIDController.calculate(this.getActualTurningPosition(), goalPosition);
        double FFVal = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity,
                targetAcceleration);

        // SmartDashboard.putNumber(m_swerveModuleName + " trgt", targetVelocity);
        // SmartDashboard.putNumber(m_swerveModuleName + " actl", actualVelocity);

        m_turningMotor.setVoltage(pidVal + FFVal);
        this.m_turningLastSpeed = actualVelocity;
        this.m_turningLastTime = Timer.getFPGATimestamp();
        this.m_turningLastPosition = this.getActualTurningPosition();

        // SmartDashboard.putNumber(this.m_swerveModuleName + " T Target Velocity",
        // targetVelocity);
        // SmartDashboard.putNumber(this.m_swerveModuleName + " T Actual Velocity",
        // actualVelocity);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees or
        // PI/2 radians
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(getActualTurningPosition()));

        double currentMPS = m_driveMotor.getEncoder().getVelocity(); // fixed with conversion

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(currentMPS,
                state.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // SmartDashboard.putNumber("desired drive speed" + m_swerveModuleName,
        // state.speedMetersPerSecond);
        // SmartDashboard.putNumber("actual drive speed" + m_swerveModuleName,
        // currentMPS);

        if (desiredState.speedMetersPerSecond < 0.1) {
            m_driveMotor.setVoltage(0);
        } else {
            m_driveMotor.setVoltage(driveOutput + driveFeedforward);
        }

        this.goToPosition(state.angle.getRadians());
        m_desiredState = state;
        
        // SmartDashboard.putNumber(m_swerveModuleName + " Drive Actual Velocity",
        // currentMPS);
        // SmartDashboard.putNumber(m_swerveModuleName + " Drive Target Velocity",
        // state.speedMetersPerSecond);
    }

    /**
     * Put info on smart dashboard
     */
    public void print() {
        // encoderOffset.setDouble(m_turningEncoder.getAbsolutePosition().getValue() * 2
        // * Math.PI);
        // absEncoderOffset.setDouble(Math.toDegrees(this.getActualTurningPosition()));

        // SmartDashboard.putNumber(m_swerveModuleName + " Actual Turning Position",
        // getActualTurningPosition());

        // SmartDashboard.putNumber(m_swerveModuleName + " Drive Output", driveOutput);
        // SmartDashboard.putNumber(m_swerveModuleName + " Turning Output", turnOutput);

        // SmartDashboard.putNumber(this.m_swerveModuleName + " T Output Voltage",
        // pidVal + FFVal);
        // SmartDashboard.putNumber(this.m_swerveModuleName + " T Target Position",
        // goalPosition);
        // SmartDashboard.putNumber(this.m_swerveModuleName + " T Turning Position",
        // this.getActualTurningPosition());
    }
}
