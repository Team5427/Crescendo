package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.DrivetrainConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.Vision.ObjectDetector;
import frc.robot.util.MiscUtil;

public class TestShooterRanging extends Command {

    private Shooter shooter;
    private SwerveDrivetrain drivetrain;
    private ProfiledPIDController rotPID;
    private ObjectDetector tagCam;

    private static final double kP = 3.25; //increase to make aggressive during static
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private static final double visionParallelPScalar = 1.0; //increase to make PID stronger during movement
    private static final double OTF_ROT_PARALLEL = 0.0; //increase to make it compensate for movement more
    private static final double OTF_ROT_DISTANCE = 0.0; //increase to make it compensate less at a distance

    public TestShooterRanging() {
        shooter = Shooter.getInstance();
        drivetrain = SwerveDrivetrain.getInstance();
        tagCam = RobotContainer.getTagCam();
        rotPID = new ProfiledPIDController(kP, kI, kD, 
            new Constraints(DrivetrainConstants.MAX_ROTATION_SPEED_RAD_S_TELEOP, DrivetrainConstants.MAX_ROTATION_SPEED_RAD_S_TELEOP * 2)
        );
        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        rotPID.setTolerance(Math.toRadians(1.5));

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        double[] targetingInformation = MiscUtil.targetingInformation();
        Rotation2d rotError = Rotation2d.fromRadians(targetingInformation[3]);

        rotPID.reset(rotError.getRadians());
    }

    @Override
    public void execute() {
        double[] targetingInformation = MiscUtil.targetingInformation();
        double parallelSpeed = targetingInformation[0];
        double perpSpeed = targetingInformation[1];
        double distance = targetingInformation[2];
        Rotation2d rotError = Rotation2d.fromRadians(targetingInformation[3]);

        Rotation2d adjustmentSetpoint = Rotation2d.fromRadians(0.0); //FIXME WHERE THE MATH IS
        ShootingConfiguration config = ShooterConstants.SHOOTER_PIVOT_TARGET_MAP.get(distance).adjustBy(
            Rotation2d.fromDegrees(perpSpeed), 
            0.0,
            0.0 
        );

        shooter.setShootingConfigSetpoints(config);
        double angleEffort = tagCam.targetVisible() ? 
            rotPID.calculate(Math.toRadians(RobotContainer.getTagCam().targetInfo()[0]), adjustmentSetpoint.getRadians()) : 
            rotPID.calculate(rotError.getRadians(), adjustmentSetpoint.getRadians());

        if (tagCam.targetVisible()) {
            rotPID.setP(Math.abs(parallelSpeed) * visionParallelPScalar + kP);
        } else {
            rotPID.setP(kP);
        }
        
        drivetrain.adjustSpeeds(new ChassisSpeeds(0, 0, 
            angleEffort - drivetrain.getSetpoint().omegaRadiansPerSecond
        ));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.adjustSpeeds(new ChassisSpeeds());
        shooter.setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM, ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
        shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_STOW);
    }
}
