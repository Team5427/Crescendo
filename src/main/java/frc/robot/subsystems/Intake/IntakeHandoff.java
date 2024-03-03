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
        intake.setLimits(60);
    }

    @Override
    public void execute() {

        if (intake.atGoal(2.0) && Shooter.getInstance().pivotAtGoal(1.0)){
            intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_EJECTING);
        }

        // if (Shooter.getInstance().loaded()) {
        //     intake.setPivotSetpoint(IntakeConstants.STOWED_POS);
        // }

    }

    @Override
    public boolean isFinished() {
        return Shooter.getInstance().inPosition();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_HOLD);
        intake.setPivotSetpoint(IntakeConstants.STOWED_POS);
        intake.setLimits(60);
    }


}
