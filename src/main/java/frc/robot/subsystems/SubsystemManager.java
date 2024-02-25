package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.BumpFeeder;
import frc.robot.subsystems.Shooter.FeedShooter;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class SubsystemManager {

    public static Command getComplexIntakeCommand() {
        return new SequentialCommandGroup(
            Intake.getInstance().getIntakeCommand(),
            new ParallelCommandGroup(
                Intake.getInstance().getIntakeHandoff(),
                Shooter.getInstance().getShooterHandoff()
            ),
            new BumpFeeder(),
            new InstantCommand(() -> {
                Shooter.getInstance().setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM, ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
            })
        );
    }
    
}
