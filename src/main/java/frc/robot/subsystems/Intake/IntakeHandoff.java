package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;

public class IntakeHandoff extends Command {
    private Intake intake;
    public IntakeHandoff() {
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPivotSetpoint(IntakeConstants.HANDOFF_POS);
    }

    @Override
    public void execute() {

        if (intake.atGoal(5.0) && Shooter.getInstance().pivotAtGoal(3.0)){
            intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_EJECTING);
        }

        if (Shooter.getInstance().loaded()) {
            System.err.println("got here so far");
            intake.setPivotSetpoint(IntakeConstants.STOWED_POS);
        }
    }

    @Override
    public boolean isFinished() {
        return !intake.sensorCovered();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_HOLD);
    }


}
