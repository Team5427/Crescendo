package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Backshot extends Command {
    private Intake intake;
    private Timer timer;
    
    public Backshot() {
        intake = Intake.getInstance();
        addRequirements(intake);
        intake.setLimits(60);
    }

    @Override
    public void initialize() {
        intake.setPivotSetpoint(IntakeConstants.BACKSHOT_POSE);
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_EJECTING_MAX);
        timer = new Timer();
    }

    @Override
    public void execute() {
        if (!intake.sensorCovered()) {
            timer.start();
        } else {
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return !intake.sensorCovered() && timer.get() > 0.5;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPivotSetpoint(IntakeConstants.STOWED_POS);
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_HOLD);
        intake.setLimits(50);
        timer.stop();
        timer.reset();
    }
}
