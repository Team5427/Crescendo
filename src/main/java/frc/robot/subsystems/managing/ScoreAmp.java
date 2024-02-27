package frc.robot.subsystems.managing;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;

public class ScoreAmp extends Command {

    Shooter shooter;
    SwerveDrivetrain drivetrain;

    public ScoreAmp() {
        shooter = Shooter.getInstance();
        drivetrain = SwerveDrivetrain.getInstance();
        addRequirements(shooter);

    }

    @Override
    public void initialize() {
        shooter.setAmpSetpoint(ShooterConstants.AMP_DEPLOYED);
        shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_AMP);
        shooter.setFlywheelSetpoint(ShooterConstants.FLYWHEEL_AMP_SPEED_RPM, ShooterConstants.FLYWHEEL_AMP_SPEED_RPM);
        drivetrain.setRotLock(Optional.of(new Rotation2d(Math.PI)));
    }

    @Override
    public boolean isFinished() {
        return !shooter.loaded();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setAmpSetpoint(ShooterConstants.AMP_HARDSTOP);
        shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_STOW);
        shooter.setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM, ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
        drivetrain.setRotLock(Optional.empty());

    }

    
}
