package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public class SubsystemManager {

    public static Command getComplexIntakeCommand() {
        return new SequentialCommandGroup(
            Intake.getInstance().getIntakeCommand(),
            new ParallelCommandGroup(
                Intake.getInstance().getIntakeHandoff(),
                Shooter.getInstance().getShooterHandoff()
            )
        );
    }
    
}
