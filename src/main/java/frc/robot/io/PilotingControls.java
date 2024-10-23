package frc.robot.io;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve.DrivetrainConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.util.MiscUtil;
import frc.robot.util.Localization.SteelTalonsLocalization;

public class PilotingControls {
    public PilotingControls(CommandXboxController controller) {

        controller.y().onTrue(new InstantCommand(() -> {
            SwerveDrivetrain.getInstance().resetGyro(new Rotation2d());
            SteelTalonsLocalization.getInstance().resetPose(
                MiscUtil.isBlue() ? MiscUtil.resetPose()[0] : MiscUtil.resetPose()[1]
            );
            SteelTalonsLocalization.getInstance().resetCameras();
        }, SteelTalonsLocalization.getInstance()));

        controller.rightBumper().onTrue(new InstantCommand(() -> {
            SwerveDrivetrain.getInstance().setDriveConfig(DrivetrainConstants.DEFAULT_DRIVE_CONFIG);
        }));

        // controller.rightBumper().whileTrue(new TestShooterRanging());

        // controller.rightTrigger(0.1).onTrue(new Command() {
        //     @Override
        //     public void initialize() {
        //         addRequirements(Climber.getInstance());
        //         Climber.getInstance().setSetpoint(ClimberConstants.CLIMB_POSITION);
        //     }
        // });

        // controller.leftTrigger(0.1 ).onTrue(Climber.getInstance().getClimbCommand());

    }
}
