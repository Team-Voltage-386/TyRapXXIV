package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Controller;
import frc.robot.Constants.Deadbands;
import frc.robot.Subsystems.Drivetrain;

public class lockTarget extends Command {
    Drivetrain dt;
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(Controller.kRateLimitXSpeed);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(Controller.kRateLimitYSpeed);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Controller.kRateLimitRot);

    public lockTarget(Drivetrain dt) {
        this.dt = dt;
        addRequirements(dt);
    }

    @Override
    public void initialize() {
        System.out.println("Locking piece.");
    }

    double xSpeed;
    double ySpeed;
    double rotSpeed;

    private void readControllers() {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        xSpeed = -m_xspeedLimiter
                .calculate(
                        MathUtil.applyDeadband(Controller.kDriveController.getLeftY(), Deadbands.kLeftJoystickDeadband))
                * Constants.Controller.kMaxNecessarySpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        ySpeed = -m_yspeedLimiter
                .calculate(
                        MathUtil.applyDeadband(Controller.kDriveController.getLeftX(), Deadbands.kLeftJoystickDeadband))
                * Constants.Controller.kMaxNecessarySpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        rotSpeed = -m_rotLimiter
                .calculate(MathUtil.applyDeadband(Controller.kDriveController.getRightX(),
                        Deadbands.kRightJoyStickDeadband))
                * Drivetrain.kMaxAngularSpeed;
    }

    @Override
    public void execute() {
        readControllers();
        dt.lockPiece(xSpeed, ySpeed, rotSpeed, !Controller.kDriveController.getHID().getAButton(),
                Controller.kDriveController.getLeftTriggerAxis() > 0.25);
    }

    @Override
    public boolean isFinished() {
        return Controller.kDriveController.getHID().getLeftBumperReleased();
    }
}
