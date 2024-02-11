package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;

public class resetOdo extends Command {
    Drivetrain dt;
    int i;

    public resetOdo(Drivetrain dt) {
        this.dt = dt;
        i = 0;
    }

    @Override
    public void schedule() {
        dt.resetGyro();
        dt.resetOdo();
    }

    @Override
    public void execute() {
        dt.resetGyro();
        dt.resetOdo();
        i++;
    }

    @Override
    public boolean isFinished() {
        return i > 1;
    }
}
