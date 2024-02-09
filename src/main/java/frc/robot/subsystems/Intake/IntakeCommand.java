package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class IntakeCommand extends Command {

    private Intake intake;

    public IntakeCommand () {
        this.intake = Intake.getInstance();

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setPivotSetpoint(IntakeConstants.INTAKING_POS);
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_INTAKING);
    }

    @Override
    public boolean isFinished() {
        return intake.sensorCovered() && intake.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPivotSetpoint(IntakeConstants.STOWED_POS);
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_STOPPED);
    }

}