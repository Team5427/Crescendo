package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TempIntakeEjaculation extends Command {

    private Intake intake;

    public TempIntakeEjaculation () {
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void execute() {
        // intake.hardSetRoller(-0.9);
        intake.setRollerSetpoint(-0.9 * IntakeConstants.ROLLER_CONFIG.maxVel);
    }

    @Override
    public boolean isFinished() {
        return !intake.sensorCovered();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_STOPPED);
        System.out.println("Ejaculated");
    }
    
}
