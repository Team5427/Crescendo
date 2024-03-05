package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.DrivetrainConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.Vision.ObjectDetector;
import frc.robot.util.MiscUtil;
import frc.robot.util.SteelTalonsLogger;

public class TestShooterRanging extends Command {

    private Shooter shooter;
    private SwerveDrivetrain drivetrain;
    private ProfiledPIDController rotPID;
    private ObjectDetector tagCam;

    private static final double kP = 1.5; //FIXME
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private static final double VISION_PARALLEL_P_SCALAR = 0.47; //increase to make PID stronger during movement
    private static final double OTF_ROT_PARALLEL = 7.5; //increase to make it compensate for parallel movement more
    //DEGREES - this value is meant for 2 meters dist

    public TestShooterRanging() {
        shooter = Shooter.getInstance();
        drivetrain = SwerveDrivetrain.getInstance();
        tagCam = RobotContainer.getTagCam();
        rotPID = new ProfiledPIDController(kP, kI, kD, new Constraints(
            DrivetrainConstants.MAX_ROTATION_SPEED_RAD_S_TELEOP,
            DrivetrainConstants.MAX_ROTATION_SPEED_RAD_S_TELEOP * 3 
        ));
        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        rotPID.setTolerance(Math.toRadians(1.5));

    }

    @Override
    public void initialize() {
        rotPID.reset(drivetrain.getRotation().getRadians());
    }

    @Override
    public void execute() {
        double[] targetingInformation = MiscUtil.targetingInformation();
        double parallelSpeed = targetingInformation[0];
        double perpSpeed = targetingInformation[1];
        double distance = targetingInformation[2];
        Rotation2d rotError = Rotation2d.fromRadians(targetingInformation[3]);
        Rotation2d translationAngle = Rotation2d.fromRadians(targetingInformation[4]);

        Rotation2d adjustmentSetpoint = new Rotation2d();
        ShootingConfiguration config;
        // ShootingConfiguration config = ShooterConstants.SHOOTER_PIVOT_TARGET_MAP.get(distance).adjustBy(
        //     new Rotation2d(), 
        //     0.0,
        //     0.0 
        // );

        // adjustmentSetpoint = rotationalOTF(parallelSpeed, distance); //FIXME WHERE THE MATH IS
        if (distance < 5.0) {
            config = ShooterConstants.SHOOTER_PIVOT_TARGET_MAP.get(distance).adjustBy(
                Rotation2d.fromDegrees(ShooterConstants.SHOOTER_OTF_OFFSET_MAP.get(perpSpeed)).
                plus(Rotation2d.fromDegrees(Math.abs(translationAngle.getDegrees()) * (4.0 / 60.0) * 0.2 * (5 - distance))), //4 degrees of offset for 60 degree angle
                0.0,
                0.0 
            );   
            
        } else {
            config = new ShootingConfiguration(
                ShooterConstants.SHOOTER_PIVOT_STOW, 
                5500, 
                5500
            );
        }

        SteelTalonsLogger.post("angle offset anglesss",(Rotation2d.fromDegrees(Math.abs(translationAngle.getDegrees()) * (3.0 / 60.0)).getDegrees()));
        SteelTalonsLogger.post("movement angle offset",Rotation2d.fromDegrees(ShooterConstants.SHOOTER_OTF_OFFSET_MAP.get(perpSpeed)).getDegrees());

        shooter.setShootingConfigSetpoints(config);
        // shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_HANDOFF);
        double angleEffort = tagCam.targetVisible() ? 
            rotPID.calculate(Math.toRadians(RobotContainer.getTagCam().targetInfo()[0]), adjustmentSetpoint.getRadians()) : 
            -rotPID.calculate(rotError.getRadians(), adjustmentSetpoint.getRadians());

        if (tagCam.targetVisible()) {
            rotPID.setP(Math.abs(parallelSpeed) * VISION_PARALLEL_P_SCALAR + kP);
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

    public Rotation2d rotationalOTF(double parallelSpeed, double distance) {
        double yVal = Math.sin(Math.toRadians(parallelSpeed * OTF_ROT_PARALLEL)) * distance;
        return new Rotation2d(distance, yVal); //basically makes it so that based on distance, the angle becomes less or more
    }
}
