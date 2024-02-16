package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShootingConfiguration;
import frc.robot.util.MiscUtil;
import frc.robot.util.SteelTalonsLogger;
import frc.robot.util.Localization.SteelTalonsLocalization;

public class TargetSpeaker extends Command {

    SwerveDrivetrain drivetrain;
    SteelTalonsLocalization localization;
    Shooter shooter;
    PIDController angPID;
    Rotation2d offsetSetpoint = new Rotation2d();

    double dynamicPivotConstant = 0.0;
    double dynamicAngleConstant = 0.0;

    public TargetSpeaker() {
        drivetrain = SwerveDrivetrain.getInstance();
        localization = SteelTalonsLocalization.getInstance();
        shooter = Shooter.getInstance();
        angPID = new PIDController(0.0, 0.0, 0.0); //FIXME
        angPID.setSetpoint(offsetSetpoint.getRadians());
        angPID.setTolerance(Units.degreesToRadians(1.5), Units.degreesToRadians(5)); //FIXME
        angPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double[] targetingInfo = MiscUtil.targetingInformation(); //parallel, perp, distance, angle

        offsetSetpoint = new Rotation2d(Units.degreesToRadians(0.0)); 
        //FIXME when move while shoot. either have to change the setpoint, or change the omega velocity (based on perp speed and distance)
        angPID.setSetpoint(offsetSetpoint.getRadians());

        ChassisSpeeds outputAdjustment = new ChassisSpeeds(0.0, 0.0, angPID.calculate(targetingInfo[3]) - drivetrain.getVelocityVector().omegaRadiansPerSecond);
        
        SteelTalonsLogger.post("Ready to Shoot", angPID.atSetpoint());

        drivetrain.adjustSpeeds(outputAdjustment);

        ShootingConfiguration staticConfig = ShooterConstants.SHOOTER_PIVOT_TARGET_MAP.get(targetingInfo[2]);

        shooter.setPivotSetpoint(staticConfig.getPivotAngle().plus(new Rotation2d(
            targetingInfo[0] * dynamicPivotConstant
        )));

        shooter.setFlywheelSetpoint(staticConfig.getLeftSpeed(), staticConfig.getRightSpeed());
    }
}
