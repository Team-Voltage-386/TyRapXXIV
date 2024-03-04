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

    private Timer timer;

    public autoPickupNote(Drivetrain dt) {
        this.dt = dt;
        this.timer = new Timer();
        this.timer.start();
        addRequirements(dt);
    }

    @Override
    public void initialize() {
        System.out.println("Locking piece.");
        this.timer.reset();
        Aimlock.setNoteVision(true);
    }

    double xSpeed;
    double ySpeed;
    double rotSpeed;

    private void readControllers() {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        xSpeed = Aimlock.hasNoteTarget() ? 1 : 0;
        // ySpeed = Aimlock.hasTarget() ?
        // MathUtil.clamp(LimelightHelpers.getTY("limelight-c") / 20, 0, 1) : 0;
    }

    @Override
    public void execute() {
        readControllers();
        dt.lockPiece(xSpeed, ySpeed, 0, true, true);
    }

    @Override
    public void end(boolean interrupted) {
        Aimlock.setNoteVision(false);
        System.out.printf("cancelled auto pickup: %b\n", interrupted);
    }

    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(2)
                // || (LimelightHelpers.getTY("limelight-c") > -5 && Aimlock.hasTarget())
                || Flags.pieceState.equals(subsystemsStates.holdingPiece);
    }
}
