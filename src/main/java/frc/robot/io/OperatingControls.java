package frc.robot.io;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.FeedShooter;
import frc.robot.subsystems.Shooter.FeedShooterClean;
import frc.robot.subsystems.Shooter.HardCodeShot;
import frc.robot.subsystems.Shooter.HomeAmp;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.TargetSpeaker;
import frc.robot.subsystems.Swerve.DrivetrainConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.managing.AutonShoot;
import frc.robot.subsystems.managing.ScoreAmp;
import frc.robot.subsystems.managing.SubsystemManager;
import frc.robot.subsystems.managing.Unstuck;
import frc.robot.util.LEDManager;
import frc.robot.util.LEDManager.LEDState;
import frc.robot.util.Localization.SteelTalonsLocalization;

public class OperatingControls {

    public OperatingControls(CommandXboxController operatingController) {
        operatingController.leftTrigger(0.1).onTrue(
                SubsystemManager.getComplexIntakeCommand().andThen(new HomeAmp()));

        operatingController.rightTrigger(0.1).whileTrue(
                Intake.getInstance().getIntakeEjaculation());

        // operatingController.leftBumper().whileTrue(new FeedShooter(5200, ShooterConstants.SHOOTER_PIVOT_ACTIVE, true));
        operatingController.leftBumper().onTrue(new HomeAmp());
        operatingController.rightBumper().onTrue(SubsystemManager.homeAll());

        operatingController.a().onTrue(new Unstuck());

        operatingController.b().whileTrue(new TargetSpeaker());
        operatingController.povDown().onTrue(new FeedShooterClean());
        operatingController.povUp().onTrue(new HardCodeShot(ShooterConstants.FIRST_AUTON_SHOT_CONFIGURATION));

        operatingController.y().whileTrue(new ScoreAmp());

        operatingController.x().whileTrue(new RunCommand(() -> {
                SwerveDrivetrain.getInstance().setDriveConfig(DrivetrainConstants.SHUTTLE_DRIVE_CONFIG);
                Shooter.getInstance().setShootingConfigSetpoints(ShooterConstants.SHUTTLE_CONFIGURATION);
                LEDManager.getInstance().setState(LEDState.kTargeting);
        }).finallyDo(() -> {
                SwerveDrivetrain.getInstance().setDriveConfig(DrivetrainConstants.DEFAULT_DRIVE_CONFIG);
                Shooter.getInstance().setShootingConfigSetpoints(ShooterConstants.DEFAULT_CONFIGURATION);
                LEDManager.getInstance().resetStates();
        }));


        operatingController.leftStick().whileTrue(new RunCommand(() -> {
                LEDManager.getInstance().setState(LEDState.kAmpSignal);
                SteelTalonsLocalization.getInstance().resetReturnPoseRot();
        }).handleInterrupt(LEDManager.getInstance()::resetStates));

        operatingController.rightStick().whileTrue(new RunCommand(() -> {
                LEDManager.getInstance().setState(LEDState.kCoopSignal);
                SteelTalonsLocalization.getInstance().resetReturnPoseRot();
        }).handleInterrupt(LEDManager.getInstance()::resetStates));

        

    }
}
