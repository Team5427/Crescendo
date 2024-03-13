package frc.robot.io;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.FeedShooter;
import frc.robot.subsystems.Shooter.FeedShooterClean;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.TestShooterRanging;
import frc.robot.subsystems.managing.AutonShoot;
import frc.robot.subsystems.managing.ScoreAmp;
import frc.robot.subsystems.managing.SubsystemManager;
import frc.robot.subsystems.managing.Unstuck;
import frc.robot.util.LEDManager;
import frc.robot.util.LEDManager.LEDState;

public class OperatingControls {

    public OperatingControls(CommandXboxController operatingController) {
        operatingController.leftTrigger(0.1).onTrue(
                SubsystemManager.getComplexIntakeCommand());

        operatingController.rightTrigger(0.1).whileTrue(
                Intake.getInstance().getIntakeEjaculation());

        operatingController.leftBumper().whileTrue(new ScoreAmp());
        operatingController.rightBumper().onTrue(SubsystemManager.homeAll());

        operatingController.a().onTrue(new Unstuck());

        operatingController.b().whileTrue(new TestShooterRanging());
        operatingController.povUp().onTrue(new FeedShooter(0, null, false));
        operatingController.povDown().onTrue(new FeedShooterClean());
        operatingController.povLeft().onTrue(new InstantCommand(() -> {
                Intake.getInstance().getPivot().resetController();
        }));
        // operatingController.x().onTrue(new ShooterHandoff());
        // operatingController.back().onTrue(SubsystemManager.pathFind()); // verify this is correct
        operatingController.y().whileTrue(new ScoreAmp());
        operatingController.x().onTrue(new AutonShoot(true));

        operatingController.leftStick().whileTrue(new RunCommand(() -> {
                LEDManager.getInstance().setState(LEDState.kAmpSignal);
                System.err.println("this thing");
        }).handleInterrupt(LEDManager.getInstance()::resetStates));

        operatingController.rightStick().whileTrue(new RunCommand(() -> {
                LEDManager.getInstance().setState(LEDState.kCoopSignal);
        }).handleInterrupt(LEDManager.getInstance()::resetStates));

        

    }
}
