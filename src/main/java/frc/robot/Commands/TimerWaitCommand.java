package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TimerWaitCommand extends Command {
    private Timer watchClockTimer;
    private final double waitSeconds;
    private boolean hasTimeElapsed;

    public TimerWaitCommand(double seconds) {
        watchClockTimer = new Timer();
        watchClockTimer.start();
        waitSeconds = seconds;
        hasTimeElapsed = false;
    }

    @Override
    public void initialize() {
        watchClockTimer.reset();
    }

    @Override
    public void execute() {
        if (watchClockTimer.advanceIfElapsed(waitSeconds)) {
            hasTimeElapsed = true;
        }
    }

    @Override
    public boolean isFinished() {
        return hasTimeElapsed;
    }
}
