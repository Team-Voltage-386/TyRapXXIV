package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Utils.Aimlock;
import frc.robot.Utils.Flags;
import frc.robot.Utils.LimelightHelpers;
import frc.robot.Utils.Flags.subsystemsStates;

public class autoPickupNote extends Command {
    Drivetrain dt;

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
        xSpeed = Aimlock.hasTarget() ? 1 : 0;
        ySpeed = Aimlock.hasTarget() ? MathUtil.clamp(LimelightHelpers.getTY("limelight-c") / 20, 0, 1) : 0;
    }

    @Override
    public void execute() {
        readControllers();
        dt.lockPiece(xSpeed, ySpeed, 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > (start + 1.25)
                || (LimelightHelpers.getTY("limelight-a") > -8 && Aimlock.hasTarget())
                || Flags.pieceState.equals(subsystemsStates.holdingPiece);
    }
}
