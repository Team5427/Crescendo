package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.util.SteelTalonsLogger;

public class IntakeHandoff extends Command {
    private Intake intake;
    public IntakeHandoff() {
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPivotSetpoint(IntakeConstants.HANDOFF_POS);
        SteelTalonsLogger.post("running outtake", false);
    }

    @Override
    public void execute() {

        // SteelTalonsLogger.post("Timer 2", timer2.get());
        if (intake.atGoal(5.0) && Shooter.getInstance().pivotAtGoal(3.0)){
            // System.err.println("intake outtaking");
            // SteelTalonsLogger.post("runnig outtake", true);
            intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_EJECTING);
        }

        if (Shooter.getInstance().loaded()) {
            intake.setPivotSetpoint(IntakeConstants.STOWED_POS);
            // System.err.println("moving intake back");
        }
    }

    @Override
    public boolean isFinished() {
        return !intake.sensorCovered();
        // return false;
    }

    @Override
    public void end(boolean interrupted) {
        // intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_HOLD);
        SteelTalonsLogger.post("running outtake", false);
    }


}
