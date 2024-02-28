package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class FeedShooterClean extends Command {

    Shooter shooter;

    public FeedShooterClean() {
        shooter = Shooter.getInstance();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_FEED_SPEED);
    }

    @Override
    public boolean isFinished() {
        return !shooter.loaded();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_HOLD_SPEED);
    }


}
