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
import frc.robot.subsystems.Shooter.TargetSpeaker;
import frc.robot.subsystems.managing.AutonShoot;
import frc.robot.subsystems.managing.ScoreAmp;
import frc.robot.subsystems.managing.SubsystemManager;
import frc.robot.subsystems.managing.Unstuck;
import frc.robot.util.LEDManager;
import frc.robot.util.LEDManager.LEDState;

public class OperatingControls {

    public OperatingControls(CommandXboxController operatingController) {
        operatingController.leftTrigger(0.1).onTrue(
                SubsystemManager.getComplexIntakeCommand(operatingController));

        operatingController.rightTrigger(0.1).whileTrue(
                Intake.getInstance().getIntakeEjaculation());

        // operatingController.leftBumper().whileTrue(new FeedShooter(5200, ShooterConstants.SHOOTER_PIVOT_ACTIVE, true));
        operatingController.leftBumper().whileTrue(Shooter.getInstance().getFeedCommand(5200, ShooterConstants.SHOOTER_PIVOT_ACTIVE));
        operatingController.rightBumper().onTrue(SubsystemManager.homeAll());

        operatingController.a().onTrue(new Unstuck());

        operatingController.b().whileTrue(new TargetSpeaker());
        operatingController.povDown().onTrue(new FeedShooterClean());
        operatingController.povUp().onTrue(new AutonShoot(false));

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
