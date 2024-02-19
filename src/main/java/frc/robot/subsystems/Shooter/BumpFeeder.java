package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class BumpFeeder extends Command {

    Timer timer;
    Shooter shooter;

    public BumpFeeder() {
        timer = new Timer();
        shooter = Shooter.getInstance();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        shooter.setFeederSetpoint(-0.25);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 0.75;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_HOLD_SPEED);
    }


}
