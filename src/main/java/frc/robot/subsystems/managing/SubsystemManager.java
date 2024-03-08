package frc.robot.subsystems.managing;

import java.sql.Driver;
import java.util.function.BooleanSupplier;

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

    public static Command getComplexIntakeCommand() {
        return new SequentialCommandGroup(
                new ConditionalCommand(
                    Intake.getInstance().getIntakeCommand().withTimeout(3.0), 
                    Intake.getInstance().getIntakeCommand(), 
                    DriverStation::isAutonomous),
                new ParallelCommandGroup(
                    Shooter.getInstance().getShooterHandoff(),
                    Intake.getInstance().getIntakeHandoff().onlyWhile(Shooter.getInstance()::notAtStow)
                ).onlyIf(Intake.getInstance()::sensorCovered),
                new ConditionalCommand(
                    new InstantCommand(() -> {
                        Shooter.getInstance().setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM, ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
                        Shooter.getInstance().setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_STOW);
                    }), 
                    new InstantCommand(() -> {
                        Shooter.getInstance().setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM, ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
                        Shooter.getInstance().setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_STOW);
                    }), 
                    new BooleanSupplier() {
                        public boolean getAsBoolean() {
                            return DriverStation.isAutonomous() && Shooter.getInstance().loaded();
                        };
                    }
                )
        ).handleInterrupt(() -> {
            Shooter.getInstance().setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM,
            ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
            Shooter.getInstance().setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_STOW);
        });
    }

    // public static Command getComplexIntakeCommand() {
    //     return new SequentialCommandGroup(
    //         new ConditionalCommand(
    //             Intake.getInstance().getIntakeCommand().withTimeout(3.0), 
    //             Intake.getInstance().getIntakeCommand(), 
    //             DriverStation::isAutonomous),
    //         new ParallelCommandGroup(
    //             Shooter.getInstance().getShooterHandoff(),
    //             Intake.getInstance().getIntakeHandoff().withTimeout(2.0)
    //         )
    //     );
    // }

    public static Command homeAll() {
        return new ParallelCommandGroup(
            new HomeIntake(),
            new HomeShooter(),
            new HomeAmp()
        );
    }   
    
    public static Command zeroAll() {
        return new InstantCommand(() -> {
            System.err.println("YIPPEE HOMED EVERYTHING");
            Shooter.getInstance().getShooterPivot().setPosition(ShooterConstants.SHOOTER_PIVOT_HARDSTOP.getRadians());
            Intake.getInstance().getPivot().setPosition(IntakeConstants.HARDSTOP_POS.getRadians());
            Shooter.getInstance().getShooterAmp().setPosition(ShooterConstants.AMP_HARDSTOP.getRadians());
        });
    }
}
