package frc.robot.io;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.BumpFeeder;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.TestShooterRanging;

public class OperatingControls {
    public OperatingControls(CommandXboxController operatingController) {
        operatingController.leftTrigger(0.1).whileTrue(
            Intake.getInstance().getBasicIntakeCommand()
        );

        operatingController.rightTrigger(0.1).whileTrue(
            Intake.getInstance().getIntakeEjaculation()
        );

        operatingController.leftBumper().onTrue(Intake.getInstance().getHomingCommand());
        operatingController.rightBumper().onTrue(Shooter.getInstance().getHomingCommand());

        operatingController.a().onTrue(new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                Shooter.getInstance().getShooterHandoff(), 
                new BumpFeeder(),
                new InstantCommand(() -> {
                    Shooter.getInstance().setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM, ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
                })
            ),
            Intake.getInstance().getIntakeHandoff()
        ));
        operatingController.y().onTrue(Shooter.getInstance().getFeedCommand(5200));
        operatingController.x().onTrue(Shooter.getInstance().getFeedCommand(1000));

        // operatingController.start().toggleOnTrue(new TestShooterRanging());
    }
}
