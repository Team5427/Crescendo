package frc.robot.subsystems.Intake;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.util.LEDManager;
import frc.robot.util.LEDManager.LEDState;

public class IntakeCommand extends Command {

    private Intake intake;
    private CommandXboxController controller;

    public IntakeCommand () {
        this.intake = Intake.getInstance();
        controller = new CommandXboxController(0);

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPivotSetpoint(IntakeConstants.INTAKING_POS);
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_INTAKING);
        LEDManager.getInstance().setState(LEDState.kIntaking);

    }

    @Override
    public void execute() {
        SwerveDrivetrain.getInstance().adjustSpeeds(
            new ChassisSpeeds(
                0, 
                RobotContainer.getNoteCam().noteDriveAdjustment(),
                0
            )
        );    
    }

    @Override
    public boolean isFinished() {
        return intake.sensorCovered() && intake.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        SwerveDrivetrain.getInstance().adjustSpeeds(new ChassisSpeeds());
        intake.setPivotSetpoint(IntakeConstants.STOWED_POS);
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_HOLD);

        LEDManager.getInstance().setState(intake.sensorCovered() ? LEDState.kIntakeFull : LEDState.kEmpty);
    }

}