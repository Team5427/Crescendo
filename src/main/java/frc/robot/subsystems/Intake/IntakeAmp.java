package frc.robot.subsystems.Intake;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.util.LEDManager;
import frc.robot.util.LEDManager.LEDState;

public class IntakeAmp extends Command {

    private Intake intake;
    private CommandXboxController controller;

    public IntakeAmp() {
        this.intake = Intake.getInstance();
        controller = new CommandXboxController(0);

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPivotSetpoint(IntakeConstants.AMPING_POS);
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_EJECTING);
        LEDManager.getInstance().setState(LEDState.kIntaking);

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return !intake.sensorCovered() && intake.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        SwerveDrivetrain.getInstance().adjustSpeeds(new ChassisSpeeds());
        intake.setPivotSetpoint(IntakeConstants.STOWED_POS);
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_HOLD);
        System.err.println("Amping interrupted - " + interrupted);

        LEDManager.getInstance().resetStates();
    }

}