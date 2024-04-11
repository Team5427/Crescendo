package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class FeedShooterClean extends Command {

    private Shooter shooter;
    private Timer timer;

    public FeedShooterClean() {
        shooter = Shooter.getInstance();
        timer = new Timer();
        // addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_FEED_SPEED);
        timer = new Timer();
        timer.reset();
        timer.start();

        // if (shooter.getShooterPivot().getSetPoint() == ShooterConstants.SHOOTER_PIVOT_STOW.getRadians()) {
        //     shooter.setFeederSetpoint(ShooterConstants.FEEDER_HOLD_SPEED);
        // }
    }

    @Override
    public void execute() {
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_FEED_SPEED);
    }

    @Override
    public boolean isFinished() {
        return (!shooter.loaded() && timer.get() > 0.5 && !new XboxController(1).povDown(null).getAsBoolean()); //removed shooter at stow as a return parameter
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_HOLD_SPEED);
        System.err.println("finishing shooter feed clean");
        timer.stop();
        timer.reset();
        timer = new Timer();
    }


}
