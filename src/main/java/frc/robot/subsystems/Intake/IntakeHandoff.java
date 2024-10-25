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
        // System.err.println("RUNNIG INTAKE HANDOFF INIT");
        intake.setPivotSetpoint(IntakeConstants.HANDOFF_POS);
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_HOLD_HANDOFF);
        intake.setLimits(55);
    }

    @Override
    public void execute() {
        // System.err.println("RUNNIG INTAKE HANDOFF EXEC");

        if (intake.atHandoff() && Shooter.getInstance().atHandoff()){
            intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_EJECTING);
        }

        if (Shooter.getInstance().loaded()) {
            // System.err.println("stowed position");
            intake.setPivotSetpoint(IntakeConstants.STOWED_POS);
        }

    }

    @Override
    public boolean isFinished() {
        return Shooter.getInstance().inPosition() || Shooter.getInstance().atStow();
    }

    @Override
    public void end(boolean interrupted) {
        // System.err.println("RUNNIG INTAKE HANDOFF END interrupted: " + interrupted);
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_AMPING / 18.0);
        intake.setPivotSetpoint(IntakeConstants.STOWED_POS);
        intake.setLimits(45);
    }


}
