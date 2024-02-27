package frc.robot.io;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.FeedShooter;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.TestShooterRanging;
import frc.robot.subsystems.managing.SubsystemManager;

public class OperatingControls {

    public OperatingControls(CommandXboxController operatingController) {
        operatingController.leftTrigger(0.1).onTrue(
                SubsystemManager.getComplexIntakeCommand());

        operatingController.rightTrigger(0.1).whileTrue(
                Intake.getInstance().getIntakeEjaculation());


        operatingController.rightBumper().onTrue(SubsystemManager.homeAll());

        operatingController.a().onTrue(new ParallelCommandGroup(
                Shooter.getInstance().getShooterHandoff(),
                Intake.getInstance().getIntakeHandoff()));
        operatingController.y()
                .onTrue(Shooter.getInstance().getFeedCommand(5200, ShooterConstants.SHOOTER_PIVOT_ACTIVE));
        operatingController.x().onTrue(Shooter.getInstance().getFeedCommand(ShooterConstants.FLYWHEEL_AMP_SPEED_RPM,
                ShooterConstants.SHOOTER_PIVOT_AMP));

        operatingController.b().whileTrue(new TestShooterRanging());
        operatingController.povUp().onTrue(new FeedShooter(0, null, false));
        // operatingController.x().onTrue(new ShooterHandoff());
        // operatingController.back().onTrue(SubsystemManager.pathFind()); // verify this is correct
    }
}
