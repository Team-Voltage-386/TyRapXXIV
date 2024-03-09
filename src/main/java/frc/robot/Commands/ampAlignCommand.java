package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Utils.Aimlock;
import frc.robot.Utils.BellController;
import frc.robot.Utils.LimelightHelpers;

public class ampAlignCommand extends Command {
    Drivetrain dt;
    PIDController xPid;

    private double xSpeed;
    private double ySpeed;

    public ampAlignCommand(Drivetrain dt) {
        this.dt = dt;
        addRequirements(dt);
        xPid = new PIDController(2, 0, 0);
    }

    @Override
    public void initialize() {
        System.out.println("Locking amp.");
    }

    private void getSpeeds() {
        ySpeed = Aimlock.hasTarget() ? MathUtil.clamp(5 / LimelightHelpers.getTY("limelight-b"), 0, 1.5) : 0;
        xSpeed = Aimlock.hasTarget() ? -MathUtil.clamp(xPid.calculate(LimelightHelpers.getTX("limelight-b")), 0, 1.5)
                : 0;
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
