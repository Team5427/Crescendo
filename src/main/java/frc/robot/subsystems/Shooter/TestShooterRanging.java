package frc.robot.subsystems.Shooter;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.util.MiscUtil;

public class TestShooterRanging extends Command {

    private Shooter shooter;
    private SwerveDrivetrain drivetrain;
    private PIDController rotPID;

    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public TestShooterRanging() {
        shooter = Shooter.getInstance();
        drivetrain = SwerveDrivetrain.getInstance();
        rotPID = new PIDController(kP, kI, kD);
        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        rotPID.setTolerance(Math.toRadians(1.5));

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        rotPID.reset();
    }

    @Override
    public void execute() {
        double[] targetingInformation = MiscUtil.targetingInformation();
        double parallelSpeed = targetingInformation[0];
        double perpSpeed = targetingInformation[1];
        double distance = targetingInformation[2];
        Rotation2d rotError = Rotation2d.fromRadians(targetingInformation[3]);

        Rotation2d adjustmentSetpoint = new Rotation2d(); //FIXME WHERE THE MATH IS
        ShootingConfiguration config = ShooterConstants.SHOOTER_PIVOT_TARGET_MAP.get(distance).adjustBy(
            new Rotation2d(), 
            0.0,
            0.0 
        );

        shooter.setShootingConfigSetpoints(config);
        drivetrain.adjustSpeeds(new ChassisSpeeds(0, 0, 
            rotPID.calculate(rotError.getRadians(), adjustmentSetpoint.getRadians()) - drivetrain.getSetpoint().omegaRadiansPerSecond
        ));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
