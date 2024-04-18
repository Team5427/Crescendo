package frc.robot.subsystems.managing;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
import frc.robot.subsystems.Shooter.TargetSpeaker;
import frc.robot.util.MiscUtil;

public class SubsystemManager {

    public static Command getComplexIntakeCommand() {
        return new SequentialCommandGroup(
                new ConditionalCommand( // Intaking Note
                    Intake.getInstance().getIntakeCommand().withTimeout(3.0).onlyIf(() -> {return !Shooter.getInstance().loaded();}), 
                    Intake.getInstance().getIntakeCommand(),                         
                    DriverStation::isAutonomous),
                new ParallelCommandGroup(
                    rumbleCommand().onlyIf(DriverStation::isTeleop),
                    Shooter.getInstance().getShooterHandoff(), //hopefully never needs this
                    Intake.getInstance().getIntakeHandoff()
                ).onlyIf(Intake.getInstance()::sensorCovered)
        );
    }

    public static Command getTargetingCommand() {
        return new SequentialCommandGroup(
            // new BumpFeederIn().withTimeout(1.0),
            new WaitUntilCommand(Shooter.getInstance()::atStow).withTimeout(1.0),
            new TargetSpeaker()
        );
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
        // Lets cum and go
        return new SequentialCommandGroup(
            new IntakeCommand().withTimeout(3.0),
            new Backshot().onlyIf(Intake.getInstance()::sensorCovered)
        );
    }

    public static Command rumbleCommand() {
        return new RunCommand(() -> {
            new XboxController(0).setRumble(RumbleType.kBothRumble, 0.8);
            new XboxController(1).setRumble(RumbleType.kBothRumble, 1.0);
        }).withTimeout(0.4).andThen(new InstantCommand(() -> {
            new XboxController(0).setRumble(RumbleType.kBothRumble, 0.0);
            new XboxController(1).setRumble(RumbleType.kBothRumble, 0.0);
        }));
    }
}
