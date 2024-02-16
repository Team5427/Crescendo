package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class FeedShooter extends Command {

    private Shooter shooter;
    private Timer timer;
    private final double timerThreshold = 1.0;

    public FeedShooter() {
        shooter = Shooter.getInstance();
        timer = new Timer();
    }

    @Override
    public void initialize() {
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_FEED_SPEED);
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > timerThreshold && !shooter.loaded();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_HOLD_SPEED);
    }
}
