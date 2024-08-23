package frc.robot.io;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeAmp;
import frc.robot.subsystems.Intake.IntakeCommand;
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
                // operatingController.leftTrigger(0.1).onTrue(
                // SubsystemManager.getComplexIntakeCommand().andThen(new HomeAmp())); // Uses
                // intake then
                // homes amp

                operatingController.leftTrigger(0.1).onTrue(new ConditionalCommand(
                                Intake.getInstance().getIntakeCommand(),
                                new ParallelCommandGroup(
                                                Intake.getInstance().getIntakeHandoff(),
                                                Shooter.getInstance().getShooterHandoff()),
                                () -> !Intake.getInstance().sensorCovered()));

                operatingController.povRight().onTrue(SubsystemManager.intakeAmpCommand());

                operatingController.rightTrigger(0.1).whileTrue(
                                Intake.getInstance().getIntakeEjaculation()); // Shoots out the note if stuck

                // operatingController.leftBumper().whileTrue(new FeedShooter(5200,
                // ShooterConstants.SHOOTER_PIVOT_ACTIVE, true));
                operatingController.leftBumper().onTrue(new HomeAmp()); // Homes amp
                operatingController.rightBumper().onTrue(SubsystemManager.homeAll()); // Homes everything

                operatingController.a().onTrue(new Unstuck()); // Preforms Unstuck action

                operatingController.b().whileTrue(new TargetSpeaker()); // Aims the shooter to the speaker
                operatingController.povDown().onTrue(new FeedShooterClean()); // Feeds the note to the shooter manually
                operatingController.povUp().onTrue(new HardCodeShot(ShooterConstants.FIRST_AUTON_SHOT_CONFIGURATION)); // Preset
                                                                                                                       // angle
                                                                                                                       // and
                                                                                                                       // shooting
                                                                                                                       // speed
                                                                                                                       // for
                                                                                                                       // shooting
                operatingController.y().onTrue(new IntakeAmp());
                // operatingController.y().whileTrue(new ScoreAmp()); // Scores into the amp

                operatingController.x().whileTrue(new RunCommand(() -> {
                        SwerveDrivetrain.getInstance().setDriveConfig(DrivetrainConstants.SHUTTLE_DRIVE_CONFIG); // Prepares
                                                                                                                 // Drivetrain
                                                                                                                 // for
                                                                                                                 // shuttling
                        Shooter.getInstance().setShootingConfigSetpoints(ShooterConstants.SHUTTLE_CONFIGURATION); // Prepates
                                                                                                                  // shooter
                                                                                                                  // for
                                                                                                                  // shuttling
                        LEDManager.getInstance().setState(LEDState.kTargeting);
                }).finallyDo(() -> {
                        SwerveDrivetrain.getInstance().setDriveConfig(DrivetrainConstants.DEFAULT_DRIVE_CONFIG); // Resets
                                                                                                                 // Drivetrain
                                                                                                                 // for
                                                                                                                 // normal
                                                                                                                 // driving
                        Shooter.getInstance().setShootingConfigSetpoints(ShooterConstants.DEFAULT_CONFIGURATION); // Resets
                                                                                                                  // Shooter
                                                                                                                  // for
                                                                                                                  // normal
                                                                                                                  // shooting
                        LEDManager.getInstance().resetStates();
                }));

                operatingController.leftStick().whileTrue(new RunCommand(() -> {
                        LEDManager.getInstance().setState(LEDState.kAmpSignal);
                        SteelTalonsLocalization.getInstance().resetReturnPoseRot(); // Resets the localization to the
                                                                                    // location before the amp was used
                }).handleInterrupt(LEDManager.getInstance()::resetStates));

                operatingController.rightStick().whileTrue(new RunCommand(() -> {
                        LEDManager.getInstance().setState(LEDState.kCoopSignal);
                        SteelTalonsLocalization.getInstance().resetReturnPoseRot(); // Resets localization
                }).handleInterrupt(LEDManager.getInstance()::resetStates));

        }
}
