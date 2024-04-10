package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class HardCodeShot extends Command {

    Shooter shooter;
    ShootingConfiguration config;
    Timer timer;

    public HardCodeShot(ShootingConfiguration config) {
        shooter = Shooter.getInstance();
        this.config = config;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShootingConfigSetpoints(config);
        timer = new Timer();
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.get() > 0.25) {
            shooter.setFeederSetpoint(ShooterConstants.FEEDER_FEED_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return !shooter.loaded();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_HOLD_SPEED);
        shooter.setShootingConfigSetpoints(ShooterConstants.DEFAULT_CONFIGURATION);
    }
}
