package frc.robot.subsystems.managing;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShootingConfiguration;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.util.MiscUtil;

public class AutonShoot extends Command {
    private Shooter shooter;
    private SwerveDrivetrain swerve;
    private boolean useVision;
    private PIDController visionPID;

    public AutonShoot(boolean useVision) {
        shooter = Shooter.getInstance();
        swerve = SwerveDrivetrain.getInstance();
        addRequirements(shooter);
        this.useVision = useVision;
        visionPID = new PIDController(2.75, 0, 0);
        visionPID.setTolerance(Math.toRadians(5.0), Math.toRadians(2));
    }

    @Override
    public void initialize() {
        visionPID.reset();

    }

    @Override
    public void execute() {
        Rotation2d translationAngle = Rotation2d.fromRadians(MiscUtil.targetingInformation()[4]);

        ShootingConfiguration config = new ShootingConfiguration(new Rotation2d(), 0, 0);

        if (useVision) {
            config = ShooterConstants.SHOOTER_PIVOT_TARGET_MAP.get(RobotContainer.getTagCam().speakerDist()).adjustBy(
                Rotation2d.fromDegrees(0.0).
                plus(Rotation2d.fromDegrees(Math.abs(translationAngle.getDegrees()) * (4.0 / 60.0) * 0.2 * (5 - RobotContainer.getTagCam().speakerDist()))), //4 degrees of offset for 60 degree angle
                0.0,
                0.0 
            );

            swerve.setSpeedsAuton(new ChassisSpeeds(0, 0, visionPID.calculate(RobotContainer.getTagCam().targetVisible() ? Math.toRadians(RobotContainer.getTagCam().targetInfo()[0]) : -MiscUtil.targetingInformation()[3], 0.0)));
        } else {
            config = new ShootingConfiguration(ShooterConstants.SHOOTER_PIVOT_ACTIVE, 5200, 5200);
        }
        shooter.setShootingConfigSetpoints(config);

        if (shooter.flywheelAtGoal() && shooter.pivotAtGoal(1.0) && (visionPID.atSetpoint() || !useVision)) {
            shooter.setFeederSetpoint(ShooterConstants.FEEDER_FEED_SPEED);
        }

    }

    @Override
    public boolean isFinished() {
        return !shooter.loaded();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_HANDOFF);
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_HOLD_SPEED);
    }
}
