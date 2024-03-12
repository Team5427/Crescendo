package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class BumpFeederIn extends Command {
    private Shooter shooter;

    public BumpFeederIn() {
        shooter = Shooter.getInstance();
    }

    @Override
    public void initialize() {
        if (!shooter.loaded()) {
            shooter.setFeederSetpoint(ShooterConstants.FEEDER_BUMP_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return shooter.loaded();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_HOLD_SPEED);
    }
}
