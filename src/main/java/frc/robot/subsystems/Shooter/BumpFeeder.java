package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class BumpFeeder extends Command {

    Timer timer;
    Shooter shooter;

    public BumpFeeder() {
        shooter = Shooter.getInstance();
    }

    @Override
    public void initialize() {
        shooter.setFeederSetpoint(-0.1);
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
