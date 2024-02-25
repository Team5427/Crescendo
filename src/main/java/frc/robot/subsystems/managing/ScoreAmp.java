package frc.robot.subsystems.managing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;

public class ScoreAmp extends Command {

    Shooter shooter;
    SwerveDrivetrain drivetrain;

    public ScoreAmp() {
        shooter = Shooter.getInstance();
        drivetrain = SwerveDrivetrain.getInstance();
        addRequirements(shooter);

    }

    
}
