package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class HomeAmp extends Command {

    private Shooter shooter;
    private Timer timer;

    private double homingTargetDegrees;
    private double homingSecondsToFinish;

    public HomeAmp() {
        shooter = Shooter.getInstance();

        // addRequirements(shooter);

        timer = new Timer();
        homingTargetDegrees = 1.0;
        homingSecondsToFinish = 0.25;
    }

    private void timerContinueHoming() {
        timer.reset();
        timer.start();
    }

    @Override
    public void initialize() {
        shooter.setHomingAmp(true);
        shooter.setFlywheelSetpoint(0.0, 0.0);
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
        return shooter.getAmpHoming() && timer.get() > homingSecondsToFinish;
    }

    @Override
    public void end(boolean interrupted) {
        System.err.println("homing amp finished");
        shooter.getShooterAmp().setPosition(ShooterConstants.AMP_HARDSTOP.getRadians());
        shooter.setAmpSetpoint(ShooterConstants.AMP_HARDSTOP);
        shooter.setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM, ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
        shooter.setHomingAmp(false);
    }
    
}
