package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Utils.Aimlock;
import frc.robot.Utils.BellController;
import frc.robot.Utils.LimelightHelpers;
import frc.robot.Utils.Aimlock.DoState;

public class ampAlignCommand extends Command {
    Drivetrain dt;
    BellController bc;

    double xSpeed;
    double ySpeed;

    public ampAlignCommand(Drivetrain dt) {
        this.dt = dt;
        addRequirements(dt);
        bc = new BellController(2, 3);
    }

    @Override
    public void initialize() {
        System.out.println("Locking amp.");
        Aimlock.setDoState(DoState.AMP);
    }

    private void getSpeeds() {
        xSpeed = Aimlock.hasTarget() ? MathUtil.clamp(LimelightHelpers.getTY("limelight-b") / 30, 0, 1.5) : 0;
        ySpeed = Aimlock.hasTarget() ? bc.calc(LimelightHelpers.getTX("limelight-b")) : 0;
    }

    @Override
    public void execute() {
        getSpeeds();
        dt.lockTarget(xSpeed, ySpeed, 0, false,
                false);
    }

    @Override
    public boolean isFinished() {
        return !Aimlock.hasTarget();
    }
}
