package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class HomeShooter extends Command {
    private Shooter shooter;
    private Timer timer;
    public boolean homed;

    public HomeShooter() {
        shooter = Shooter.getInstance();
        addRequirements(shooter);

        timer = new Timer();
    }

    @Override
    public void initialize() {
        shooter.movePivot(0.05);
        shooter.moveAmp(0.05);
        timerContinueHoming();
    }

    private void timerContinueHoming() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (Math.abs(shooter.getPivot().getEncoderVelocity()) > Units.degreesToRadians(0.5)) {
            timerContinueHoming();
        }
    }

    @Override
    public boolean isFinished() {
        if (shooter.pivotAtGoal() && shooter.ampAtGoal() && timer.get() > 0.5) {
            timer.stop();
            shooter.home();
            timer.reset();
            return true;
        }
        return false;
    }

}
