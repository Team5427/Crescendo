package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;

public class IntakeHandoff extends Command {
    private Intake intake;
    private Timer timer;
    private final double threshholdValSec = 0.75;
    public IntakeHandoff() {
        intake = Intake.getInstance();
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        intake.setPivotSetpoint(IntakeConstants.HANDOFF_POS);
    }

    @Override
    public void execute() {
        if (intake.atGoal(5.0) && Shooter.getInstance().pivotAtGoal(2.0)) {
            intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_EJECTING);
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= threshholdValSec;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPivotSetpoint(IntakeConstants.STOWED_POS);
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_HOLD);
    }


}
