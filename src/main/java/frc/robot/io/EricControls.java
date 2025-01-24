package frc.robot.io;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake.Backshot;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.TempIntakeEjaculation;
import frc.robot.subsystems.Shooter.FeedShooterClean;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.managing.SubsystemManager;
import frc.robot.util.MiscUtil;
import frc.robot.util.Localization.SteelTalonsLocalization;

public class EricControls {
    
    public EricControls(CommandXboxController joy) {

        joy.povUp().onTrue(new InstantCommand(() -> {
            SwerveDrivetrain.getInstance().resetGyro(new Rotation2d());
            SteelTalonsLocalization.getInstance().resetPose(
                MiscUtil.isBlue() ? MiscUtil.resetPose()[0] : MiscUtil.resetPose()[1]
            );
            SteelTalonsLocalization.getInstance().resetCameras();
        }, SteelTalonsLocalization.getInstance()));

        joy.leftTrigger(0.1).onTrue(
            SubsystemManager.getComplexIntakeCommand().onlyIf(() -> !Intake.getInstance().sensorCovered())
        );

        joy.rightTrigger(0.1).onTrue(
            new SequentialCommandGroup(
                new RunCommand(() -> {
                    Shooter.getInstance().setFlywheelSetpoint(5300, 5300);
                    Shooter.getInstance().setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_SHUTTLE);
                }).withTimeout(0.25).andThen(new FeedShooterClean()),
                new InstantCommand(() -> {
                    Shooter.getInstance().setFlywheelSetpoint(1000, 1000);
                    Shooter.getInstance().setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_STOW);
                })
            )
        );

        joy.rightBumper().onTrue(SubsystemManager.homeAll());
        joy.a().onTrue(new TempIntakeEjaculation());

    }

}
