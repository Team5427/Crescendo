package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;

public class IntakeHandoff extends Command {
    private Intake intake;
    private Timer timer;
    private Timer timer2;
    public IntakeHandoff() {
        intake = Intake.getInstance();
        addRequirements(intake);
        timer = new Timer();
        timer2 = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer2.reset();
        timer2.start();
        intake.setPivotSetpoint(IntakeConstants.HANDOFF_POS);
    }

    @Override
    public void execute() {
        if (intake.atGoal(5.0) && Shooter.getInstance().pivotAtGoal(2.0) && timer2.get() > 0.5) {
            intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_EJECTING);
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPivotSetpoint(IntakeConstants.STOWED_POS);
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_HOLD);
    }


}
