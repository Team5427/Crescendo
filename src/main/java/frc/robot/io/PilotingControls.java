package frc.robot.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.util.MiscUtil;
import frc.robot.util.Localization.SteelTalonsLocalization;

public class PilotingControls {
    public PilotingControls(CommandXboxController controller) {

        controller.y().onTrue(new InstantCommand(() -> {
            SteelTalonsLocalization.getInstance().resetPose(
                MiscUtil.isBlue() ? MiscUtil.resetPose()[0] : MiscUtil.resetPose()[1]
            );
            SwerveDrivetrain.getInstance().resetGyro(new Rotation2d());
        }, SteelTalonsLocalization.getInstance()));

        

    }
}
