package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class TempIntakeEjaculation extends Command {

    private Intake intake;

    public TempIntakeEjaculation () {
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPivotSetpoint(IntakeConstants.INTAKING_POS);
    }

    @Override
    public void execute() {
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_EJECTING);
    }

    @Override
    public boolean isFinished() {
        return !intake.sensorCovered();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_HOLD);
        intake.setPivotSetpoint(IntakeConstants.STOWED_POS);
    }
    
}
