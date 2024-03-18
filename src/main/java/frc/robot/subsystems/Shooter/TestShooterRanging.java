package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.DrivetrainConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.Vision.ObjectDetector;
import frc.robot.util.LEDManager;
import frc.robot.util.MiscUtil;
import frc.robot.util.SteelTalonsLogger;
import frc.robot.util.LEDManager.LEDState;

public class TestShooterRanging extends Command {

    private Shooter shooter;
    private SwerveDrivetrain drivetrain;
    private PIDController rotPID;
    private ObjectDetector tagCam;

    private static final double kP = 1.75; //FIXME
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private static final double VISION_PARALLEL_P_SCALAR = 0.75; //increase to make PID stronger during movement
    private static final double OTF_ROT_PARALLEL = 7.5; //increase to make it compensate for parallel movement more
    //DEGREES - this value is meant for 2 meters dist

    public TestShooterRanging() {
        shooter = Shooter.getInstance();
        drivetrain = SwerveDrivetrain.getInstance();
        tagCam = RobotContainer.getTagCam();
        rotPID = new PIDController(kP, kI, kD);
        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        rotPID.setTolerance(Math.toRadians(1.5));

    }

    @Override
    public void initialize() {

        LEDManager.getInstance().setState(LEDState.kTargeting);
    }

    @Override
    public void execute() {
        double[] targetingInformation = MiscUtil.targetingInformation();
        double parallelSpeed = targetingInformation[0];
        double perpSpeed = targetingInformation[1];
        double distance = RobotContainer.getTagCam().targetVisible() ? RobotContainer.getTagCam().speakerDist() : targetingInformation[2];
        Rotation2d rotError = Rotation2d.fromRadians(targetingInformation[3]);
        Rotation2d translationAngle = Rotation2d.fromRadians(targetingInformation[4]);

        Rotation2d adjustmentSetpoint = new Rotation2d();
        ShootingConfiguration config;

        adjustmentSetpoint = rotationalOTF(parallelSpeed, distance); //FIXME WHERE THE MATH IS
        if (distance < 7.0 || !tagCam.targetVisible()) {
            config = ShooterConstants.SHOOTER_PIVOT_TARGET_MAP.get(distance).adjustBy(
                Rotation2d.fromDegrees(ShooterConstants.SHOOTER_OTF_OFFSET_MAP.get(perpSpeed)).
                plus(Rotation2d.fromDegrees(Math.abs(translationAngle.getDegrees()) * (4.0 / 60.0) * 0.2 * (5 - distance))), //4 degrees of offset for 60 degree angle
                0.0,
                0.0 
            );   
            
        } else {
            config = new ShootingConfiguration( //EMERGENCY IF CANT SEE TARGET
                ShooterConstants.SHOOTER_PIVOT_ACTIVE, 
                5300, 
                5300
            );
        }


        SteelTalonsLogger.post("On the fly adjustment", adjustmentSetpoint.getRadians());

        shooter.setShootingConfigSetpoints(config);
        // shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_HANDOFF);
        double angleEffort = tagCam.targetVisible() ? 
            rotPID.calculate(Math.toRadians(RobotContainer.getTagCam().targetInfo()[0]), adjustmentSetpoint.getRadians()) : 
            -rotPID.calculate(rotError.getRadians(), adjustmentSetpoint.getRadians());

        if (tagCam.targetVisible()) {
            rotPID.setP(Math.abs(parallelSpeed) * VISION_PARALLEL_P_SCALAR + kP);
        } else {
            rotPID.setP(Math.abs(parallelSpeed) * VISION_PARALLEL_P_SCALAR + kP);
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

        LEDManager.getInstance().resetStates();
    }

    public Rotation2d rotationalOTF(double parallelSpeed, double distance) {
        double yVal = Math.sin(Math.toRadians(parallelSpeed * OTF_ROT_PARALLEL)) * distance;
        return new Rotation2d(distance, yVal); //basically makes it so that based on distance, the angle becomes less or more
    }
}
