package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class FeedShooter extends Command {

    private Shooter shooter;
    private Timer timer;
    private Timer timer2;
    private final double timerThreshold = 0.75;
    private double setpoint = 0.0;

    public FeedShooter(double setpoint) {
        shooter = Shooter.getInstance();
        timer = new Timer();
        timer2 = new Timer();

        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer2.reset();
        timer2.start();

        shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_AMP);
    }

    @Override
    public void execute() {
        shooter.setFlywheelSetpoint(setpoint, setpoint);
        if (shooter.pivotAtGoal() && shooter.flywheelAtGoal() && timer2.get() > 0.25) {
            System.err.println("starting the flywheel");
            shooter.setFeederSetpoint(ShooterConstants.FEEDER_FEED_SPEED);
            timer.start();
        } else {
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.get() > timerThreshold && !shooter.loaded();
    }

    @Override
    public void end(boolean interrupted) {
        System.err.println("finishing fefed");
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_HOLD_SPEED);
        shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_STOW);
        shooter.setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM, ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
        timer.stop();
        timer2.stop();
    }
}
