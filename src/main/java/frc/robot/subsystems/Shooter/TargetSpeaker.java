package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.Vision.ObjectDetector;
import frc.robot.util.LEDManager;
import frc.robot.util.MiscUtil;
import frc.robot.util.SteelTalonsLogger;
import frc.robot.util.LEDManager.LEDState;

public class TargetSpeaker extends Command {

    private Shooter shooter;
    private SwerveDrivetrain drivetrain;
    private PIDController rotPID;
    private ObjectDetector tagCam;

    private static final double kP = 5.5; //FIXME
    private static final double kI = 0.0;
    private static final double kD = 0.15;

    private static final double VISION_PARALLEL_P_SCALAR = 0.7; //increase to make PID stronger during movement
    private static final double OTF_ROT_PARALLEL = 5.0; //increase to make it compensate for parallel movement more
    //DEGREES - this value is meant for 2 meters dist\\

    public TargetSpeaker() {
        shooter = Shooter.getInstance();
        drivetrain = SwerveDrivetrain.getInstance();
        tagCam = RobotContainer.getTagCam();
        rotPID = new PIDController(kP, kI, kD);
        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        rotPID.setTolerance(Math.toRadians(4.0), Math.toRadians(2.0));
        addRequirements(shooter);

    }

    @Override
    public void initialize() {

        LEDManager.getInstance().setState(LEDState.kTargeting);
        rotPID.reset();
    }

    @Override
    public void execute() {
        double[] targetingInformation = MiscUtil.targetingInformation();
        double parallelSpeed = targetingInformation[0];
        double perpSpeed = targetingInformation[1];
        double distance = targetingInformation[2];
        Rotation2d rotError = Rotation2d.fromRadians(targetingInformation[3]);
        Rotation2d translationAngle = Rotation2d.fromRadians(targetingInformation[4]);

        double angleEffort = 0.0;

        Rotation2d adjustmentSetpoint = new Rotation2d();
        ShootingConfiguration config;

        adjustmentSetpoint = rotationalOTF(parallelSpeed, distance); //FIXME WHERE THE MATH IS WOW
        if (distance < 7.0) {
            config = ShooterConstants.SHOOTER_PIVOT_TARGET_MAP.get(distance).adjustBy(
                Rotation2d.fromDegrees(ShooterConstants.SHOOTER_OTF_OFFSET_MAP.get(perpSpeed)).
                minus(Rotation2d.fromDegrees(Math.abs(translationAngle.getDegrees()) * ((-2.0) / 60.0) * 0.2 * (5 - distance))), //4 degrees of offset for 60 degree angle
                0.0,
                0.0 
            );   

            angleEffort = tagCam.targetVisible() ? 
            rotPID.calculate(Math.toRadians(RobotContainer.getTagCam().targetInfo()[0]), adjustmentSetpoint.getRadians()) : 
            -rotPID.calculate(rotError.getRadians(), adjustmentSetpoint.getRadians());    
            
        } else {
            config = ShooterConstants.SHUTTLE_CONFIGURATION;
            angleEffort = 0;
        }

        SteelTalonsLogger.post("On the fly adjustment", adjustmentSetpoint.getRadians());

        shooter.setShootingConfigSetpoints(config);
        // shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_HANDOFF);

        rotPID.setP(MiscUtil.drivetrainSpeedMagnitude() * VISION_PARALLEL_P_SCALAR + kP);

        if (
            (DriverStation.isTeleop() && new XboxController(0).getLeftTriggerAxis() < 0.5 && distance < 7.00) ||
            (DriverStation.isAutonomousEnabled() && MiscUtil.drivetrainSpeedMagnitude() < 1.5)
        ) {
            drivetrain.adjustSpeeds(new ChassisSpeeds(0, 0, 
                angleEffort - drivetrain.getSetpoint().omegaRadiansPerSecond
            ));
        }

        if (
            (DriverStation.isAutonomousEnabled() && 
            shooter.pivotAtGoal(0.5) && 
            shooter.pivotAtVelGoal(0.5) &&
            shooter.flywheelAtGoal() && 
            rotPID.atSetpoint())
        ) {
            shooter.setFeederSetpoint(ShooterConstants.FEEDER_FEED_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        if (DriverStation.isAutonomous()) {
            return !shooter.loaded();
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.adjustSpeeds(new ChassisSpeeds());
        shooter.setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM, ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
        shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_STOW);
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_HOLD_SPEED);

        LEDManager.getInstance().resetStates();
    }

    public Rotation2d rotationalOTF(double parallelSpeed, double distance) {
        double yVal = Math.sin(Math.toRadians(parallelSpeed * OTF_ROT_PARALLEL)) * distance;
        return new Rotation2d(distance, yVal); //basically makes it so that based on distance, the angle becomes less or more
    }
}
