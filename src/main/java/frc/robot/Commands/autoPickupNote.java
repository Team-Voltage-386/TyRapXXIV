package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Controller;
import frc.robot.Constants.Deadbands;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Utils.Aimlock;
import frc.robot.Utils.LimelightHelpers;

public class autoPickupNote extends Command {
    Drivetrain dt;
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(Controller.kRateLimitXSpeed);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(Controller.kRateLimitYSpeed);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Controller.kRateLimitRot);

    private double start;

    public autoPickupNote(Drivetrain dt) {
        this.dt = dt;
        addRequirements(dt);

    }

    @Override
    public void initialize() {
        System.out.println("Locking piece.");
        start = Timer.getFPGATimestamp();
    }

    double xSpeed;
    double ySpeed;
    double rotSpeed;

    private void readControllers() {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        xSpeed = Aimlock.hasTarget() ? 2 : 0;
    }

    @Override
    public void execute() {
        readControllers();
        dt.lockPiece(xSpeed, 0, 0, false, true);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > (start + 1.25)
                || (LimelightHelpers.getTY("limelight-a") > -8 && Aimlock.hasTarget());
    }
}
