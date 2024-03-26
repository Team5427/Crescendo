package frc.robot.subsystems.managing;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake.Backshot;
import frc.robot.subsystems.Intake.HomeIntake;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeCommand;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Shooter.BumpFeederIn;
import frc.robot.subsystems.Shooter.HomeAmp;
import frc.robot.subsystems.Shooter.HomeShooter;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class SubsystemManager {

    public static Command getComplexIntakeCommand(CommandXboxController operatingController) {
        return new SequentialCommandGroup(
                new ConditionalCommand( // Intaking Note
                    Intake.getInstance().getIntakeCommand().withTimeout(3.0), 
                    Intake.getInstance().getIntakeCommand(), 
                    DriverStation::isAutonomous).onlyIf(() -> {return !Shooter.getInstance().loaded();}),
                new ParallelCommandGroup(
                    Shooter.getInstance().getShooterHandoff(), //hopefully never needs this
                    Intake.getInstance().getIntakeHandoff().onlyWhile(Shooter.getInstance()::notAtStow)
                ).onlyIf(Intake.getInstance()::sensorCovered),
                new ConditionalCommand( // Rev up flywheels to static or auton speeds
                    new InstantCommand(() -> {
                        Shooter.getInstance().setFlywheelSetpoint(ShooterConstants.FLYWHEEL_AUTON_SPEED_RPM, ShooterConstants.FLYWHEEL_AUTON_SPEED_RPM);
                        Shooter.getInstance().setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_STOW);
                    }), 
                    new InstantCommand(() -> {
                        Shooter.getInstance().setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM, ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
                        Shooter.getInstance().setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_STOW);
                    }),
                    () -> {return DriverStation.isAutonomous();}
                )
        ).handleInterrupt(() -> {
            Shooter.getInstance().setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM,
            ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
            Shooter.getInstance().setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_STOW);
        });
    }

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

    public static Command cumAndGo() {
        return new SequentialCommandGroup(
            new IntakeCommand().withTimeout(3.0),
            new Backshot().onlyIf(Intake.getInstance()::sensorCovered)
        );
    }
}
