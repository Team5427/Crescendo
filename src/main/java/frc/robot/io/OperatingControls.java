package frc.robot.io;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake.Intake;

public class OperatingControls {
    public OperatingControls(CommandXboxController operatingController) {
        operatingController.rightTrigger(0.1).whileTrue(
            Intake.getInstance().getBasicIntakeCommand(operatingController)
        );
    }
}
