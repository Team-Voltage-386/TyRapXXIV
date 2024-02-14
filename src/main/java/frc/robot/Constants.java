package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.Subsystems.Drivetrain;

public class Constants {
    public static class ID {
        // Swerve motor controller IDs
        public static final int kFrontLeftTurn = 1;
        public static final int kFrontLeftDrive = 2;

        public static final int kFrontRightTurn = 3;
        public static final int kFrontRightDrive = 4;

        public static final int kBackRightTurn = 5;
        public static final int kBackRightDrive = 6;

        public static final int kBackLeftTurn = 7;
        public static final int kBackLeftDrive = 8;

        // Swerve CanCoder IDs
        public static final int kFrontLeftCANCoder = 9;
        public static final int kFrontRightCANCoder = 10;
        public static final int kBackRightCANCoder = 11;
        public static final int kBackLeftCANCoder = 12;

        // Pigeon
        public static final int kGyro = 13;

        // Shooter motor controller IDs
        public static final int kShooterAimMotorID = 14;
        public static final int kShooterMotorID = 15;
        public static final int kRollerMotorID = 16;
    }

    public static class Deadbands {
        public static final double kLeftJoystickDeadband = 0.1;
        public static final double kRightJoyStickDeadband = 0.1;
    }

    public static class PipeLineID { //todo update these to actually use them in game
        public static final int kNoteID = 1;
        public static final int kSpeakerID = 0;
        public static final int kAmpID = 2;
        public static final int kSourceID = 3;
    }

    public static class Shooter {
        public static final double kShooterSpeed = 10;
        public static final double kMaxAngle = 52;
        public static final double kMinAngle = 32;
        public static final double kRollerRPM = 500;
    }

    public static class DriveTrain {
        public static final double kDistanceMiddleToFrontMotor = 0.314325;
        public static final double kDistanceMiddleToSideMotor = 0.314325;
        public static final double kDriveBaseRadius = Math.sqrt( //distance from the middle to the furthest wheel
        kDistanceMiddleToFrontMotor*kDistanceMiddleToFrontMotor +
        kDistanceMiddleToSideMotor*kDistanceMiddleToSideMotor);

        public static final int kXForward = 1;
        public static final int kXBackward = -1;
        public static final int kYLeft = 1;
        public static final int kYRight = -1;

        public static final double kTranslationPathPlannerP = 5; //shouldnt need anything other than P
        public static final double kRotationPathPlannerP = 4.5;
        //ITS TUNED. NO TOUCH!
        public static final double[] turnPID = { 0, 0.0, 0.0 };
        public static final double[] drivePID = { 0.0, 0.00, 0.00 };
        public static final double[] turnFeedForward = { 0.0, 0.35 };
        public static final double[] driveFeedForward = { 0.0, 0 };

        /*
         * public static final double[] turnPID = { 5, 1.0, 0.0 };
        public static final double[] drivePID = { 0.5, 0.11, 0.11 };
        public static final double[] turnFeedForward = { 0.0, 0.45 };
        public static final double[] driveFeedForward = { 0.0, 2.74 };
         */
    };

    public static final class Modules {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75; // check 0 to auto for this (lucas will understand)
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    }

    public static class Controller {
        public static final int kDriveController = 0;
        public static final int kManipController = 1;

        /**
         * Rate limiters make joystick inputs more gentle; 1/3 sec from 0 to 1.
         */
        public static final double kRateLimitXSpeed = 100.0;
        public static final double kRateLimitYSpeed = 100.0;
        public static final double kRateLimitRot = 100.0;
        public static final double kMaxNecessarySpeed = Drivetrain.kMaxPossibleSpeed * 0.8;
    }

    public static class Offsets {
        // Ensure that the encoder offsets are between -Pi & Pi
        /**
         * Encoder offsets
         */
        public static final double kFrontLeftOffset = -0.43;
        public static final double kFrontRightOffset = -1.61;
        public static final double kBackLeftOffset = -1.53;
        public static final double kBackRightOffset = 0.86;
    }
}
