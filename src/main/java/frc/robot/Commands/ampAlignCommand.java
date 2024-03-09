package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Controller;
import frc.robot.Constants.Deadbands;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Utils.Aimlock;
import frc.robot.Utils.LimelightHelpers;

public class ampAlignCommand extends Command {
    Drivetrain dt;

    private double xSpeed;
    private double ySpeed;

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(Controller.kRateLimitXSpeed);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(Controller.kRateLimitYSpeed);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Controller.kRateLimitRot);

    public ampAlignCommand(Drivetrain dt) {
        this.dt = dt;
        addRequirements(dt);
    }

    @Override
    public void initialize() {
        System.out.println("Locking amp.");
    }

    private void getSpeeds() {
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
    }

    @Override
    public void execute() {
        getSpeeds();
        dt.lockTarget(xSpeed, ySpeed, 0, true,
                false);
    }

    @Override
    public boolean isFinished() {
        return !Aimlock.hasTarget();
    }
}
