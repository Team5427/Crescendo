package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TempIntakeEjaculation extends Command {

    private Intake intake;
    private Timer timer;

    public TempIntakeEjaculation () {
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_EJECTING);
        // timer.reset();
    }

    @Override
    public boolean isFinished() {
        return !intake.sensorCovered();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_HOLD);
        System.out.println("Ejaculated");
    }
    
    public void soloEjaculation(String person, int times) {
        if (times == 1) {
            System.out.println(person + " has ejaculated" + times + " time.");
        } else {
            System.out.println(person + " has ejaculated" + times + " times.");
        }
    }

    public void teamEjaculation(String person1, String person2, int times) {
        ericEjaculates();
        if (times == 1) {
            System.out.println(person1 + "  has ejaculated " + person2 + times + " time.");
        } else {
            System.out.println(person1 + "  has ejaculated " + person2 + times + " times.");
        }
    }
    public void ericEjaculates()
    {
        System.out.println("Eric ejaculates on the entire 5427 team!");
    }

    public void noConsentEjaculation(String person1, String person2, int times) {
        if (times == 1) {
            System.out.println(person1 + "  has ejaculated on " + person2 + times + " time.");
        } else {
            System.out.println(person1 + "  has ejaculated on " + person2 + times + " times.");
        }
    }

    // public void 
}
