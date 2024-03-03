package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class HomeShooter extends Command {

    private Shooter shooter;
    private Timer timer;

    private double homingTargetDegrees;
    private double homingSecondsToFinish;

    public HomeShooter () {
        shooter = Shooter.getInstance();

        addRequirements(shooter);

        timer = new Timer();
        homingTargetDegrees = 0.5;
        homingSecondsToFinish = 0.5;
    }

    private void timerContinueHoming() {
        timer.reset();
        timer.start();
    }

    @Override
    public void initialize() {
        shooter.setHomingPivot(true);
        shooter.setFlywheelSetpoint(0.0, 0.0);
        shooter.setFeederSetpoint(0.0);
        timerContinueHoming();
    }

    @Override
    public void execute() {
        if (Math.abs(shooter.getShooterPivot().getVelocity()) > Units.degreesToRadians(homingTargetDegrees)) {
            timerContinueHoming();
        }
    }

    @Override
    public boolean isFinished() {
        return shooter.getHoming() && timer.get() > homingSecondsToFinish;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.getShooterPivot().setPosition(ShooterConstants.SHOOTER_PIVOT_HARDSTOP.getRadians());
        shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_STOW);
        shooter.setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM, ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
        shooter.setHomingPivot(false);
    }
    
}
