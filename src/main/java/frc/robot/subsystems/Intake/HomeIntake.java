package frc.robot.subsystems.Intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class HomeIntake extends Command {

    private Intake intake;
    private Timer timer;

    private double homingTargetDegrees;
    private double homingSecondsToFinish;

    public HomeIntake () {
        intake = Intake.getInstance();

        addRequirements(intake);

        timer = new Timer();
        homingTargetDegrees = 1.0;
        homingSecondsToFinish = 0.5;
    }

    private void timerContinueHoming() {
        timer.reset();
        timer.start();
    }

    @Override
    public void initialize() {
        intake.setHoming(true);
        timerContinueHoming();
    }

    @Override
    public void execute() {
        if (Math.abs(intake.getPivot().getVelocity()) > Units.degreesToRadians(homingTargetDegrees)) {
            timerContinueHoming();
        }
    }

    @Override
    public boolean isFinished() {
        return intake.getHoming() && timer.get() > homingSecondsToFinish;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setHoming(false);
        intake.resetPivotEncoder(IntakeConstants.HARDSTOP_POS);
        intake.setPivotSetpoint(IntakeConstants.STOWED_POS);
    }
    
}
