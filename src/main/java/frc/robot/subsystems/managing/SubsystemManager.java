package frc.robot.subsystems.managing;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake.HomeIntake;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.FeedShooter;
import frc.robot.subsystems.Shooter.HomeAmp;
import frc.robot.subsystems.Shooter.HomeShooter;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.TestShooterRanging;

public class SubsystemManager {

    public static Command getComplexIntakeCommand() {
        return new SequentialCommandGroup(
                Intake.getInstance().getIntakeCommand(),
                new ParallelCommandGroup(
                        Intake.getInstance().getIntakeHandoff(),
                        Shooter.getInstance().getShooterHandoff().asProxy()
                ),
                new InstantCommand(() -> {
                    Shooter.getInstance().setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM,
                            ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
                }));
    }

    public static Command autonStaticShootCommand() {
        return new ParallelDeadlineGroup(
                new FeedShooter(0, new Rotation2d(), false),
                new TestShooterRanging());
    }

    public static Command homeAll() {
        return new ParallelCommandGroup(
            new HomeIntake(),
            new HomeShooter(),
            new HomeAmp()
        );
    }    
}
