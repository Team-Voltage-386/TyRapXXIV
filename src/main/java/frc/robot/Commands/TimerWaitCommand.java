package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TimerWaitCommand extends Command {
    private Timer watchClockTimer;
    private final double waitSeconds;

    public TimerWaitCommand(double seconds) {
        watchClockTimer = new Timer();
        watchClockTimer.start();
        waitSeconds = seconds;
    }

    @Override
    public void initialize() {
        watchClockTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return watchClockTimer.advanceIfElapsed(waitSeconds);
    }
}
