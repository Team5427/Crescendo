package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class IntakeCommand extends Command{

    private Intake intake;
    private CommandXboxController joy;

    public IntakeCommand(CommandXboxController joy) {
        this.intake = Intake.getInstance();
        this.joy = joy;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (joy.getHID().getRightBumper()) 
            intake.setMotors(.5);
    }

    @Override
    public boolean isFinished() {
        return !joy.getHID().getRightBumper();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopMotors();
    }
    
}
