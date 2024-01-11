package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveCommand extends Command {

    private CommandXboxController controller;
    private SwerveDrivetrain swerve;

    public DriveCommand(SwerveDrivetrain swerve, CommandXboxController controller) {
        addRequirements(swerve);
        this.controller = controller;
        this.swerve = swerve;
    }

    @Override
    public void execute() { //ADD ALL TELEOP MANIPULATION USING speeds.plus()
        ChassisSpeeds speeds = swerve.getDriveSpeeds(controller);
        swerve.setSetpoint(speeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
