package frc.robot.subsystems.managing;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
// import frc.robot.subsystems.Shooter.HomeAmp;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Swerve.DriveConfig;
import frc.robot.subsystems.Swerve.DrivetrainConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;

public class ScoreAmp extends Command {

    Shooter shooter;
    SwerveDrivetrain drivetrain;
    Intake intake;

    public ScoreAmp() {
        shooter = Shooter.getInstance();
        drivetrain = SwerveDrivetrain.getInstance();
        intake = Intake.getInstance();
        addRequirements(shooter, intake);

    }

    @Override
    public void initialize() {
        shooter.setAmpSetpoint(ShooterConstants.AMP_DEPLOYED);
        shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_AMP);
        shooter.setFlywheelSetpoint(ShooterConstants.FLYWHEEL_AMP_SPEED_RPM, ShooterConstants.FLYWHEEL_AMP_SPEED_RPM);
        drivetrain.setDriveConfig(DrivetrainConstants.AMP_DRIVE_CONFIG);
        intake.setPivotSetpoint(IntakeConstants.HANDOFF_POS.minus(Rotation2d.fromDegrees(9)));
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_INTAKING / 3);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setAmpSetpoint(ShooterConstants.AMP_HARDSTOP);
        shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_STOW);
        shooter.setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM, ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
        drivetrain.setDriveConfig(DrivetrainConstants.DEFAULT_DRIVE_CONFIG);
        intake.setPivotSetpoint(IntakeConstants.STOWED_POS);
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_HOLD);
        SwerveDrivetrain.getInstance().setDriveConfig(DrivetrainConstants.DEFAULT_DRIVE_CONFIG);

    }

    
}
