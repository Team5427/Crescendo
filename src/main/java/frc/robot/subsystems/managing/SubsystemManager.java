package frc.robot.subsystems.managing;

import java.sql.Driver;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake.HomeIntake;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Shooter.FeedShooter;
import frc.robot.subsystems.Shooter.HomeAmp;
import frc.robot.subsystems.Shooter.HomeShooter;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.TestShooterRanging;

public class SubsystemManager {

    // public static Command getComplexIntakeCommand() {
    //     return new SequentialCommandGroup(
    //             new ConditionalCommand(
    //                 Intake.getInstance().getIntakeCommand(), 
    //                 Intake.getInstance().getIntakeCommand(), 
    //                 DriverStation::isAutonomous),
    //             new ParallelCommandGroup(
    //                 Shooter.getInstance().getShooterHandoff().asProxy(),
    //                 Intake.getInstance().getIntakeHandoff()
    //             ),
    //             new InstantCommand(() -> {
    //                 Shooter.getInstance().setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM,
    //                         ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
    //                 Shooter.getInstance().setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_STOW);
    //             }));
    // }

    public static Command getComplexIntakeCommand() {
        return new SequentialCommandGroup(
            new ConditionalCommand(
                Intake.getInstance().getIntakeCommand().withTimeout(3.0), 
                Intake.getInstance().getIntakeCommand(), 
                DriverStation::isAutonomous),
            new ParallelCommandGroup(
                Shooter.getInstance().getShooterHandoff().asProxy(),
                Intake.getInstance().getIntakeHandoff()
            )
        );
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
