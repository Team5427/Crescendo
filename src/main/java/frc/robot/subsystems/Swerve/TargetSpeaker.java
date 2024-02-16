package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Localization.SteelTalonsLocalization;

public class TargetSpeaker extends Command {

    SwerveDrivetrain drivetrain;
    SteelTalonsLocalization localization;

    public TargetSpeaker() {
        drivetrain = SwerveDrivetrain.getInstance();
        localization = SteelTalonsLocalization.getInstance();
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        Transform2d transform = localization.transformFromSpeaker();
        double distance = transform.getTranslation().getNorm();
        Rotation2d angError = transform.getTranslation().getAngle().minus(localization.getPose().getRotation());
        ChassisSpeeds botSpeeds = drivetrain.getVelocityVector();
        ChassisSpeeds botSpeedsFieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(botSpeeds, localization.getPose().getRotation());

        ChassisSpeeds outputAdjustment = new ChassisSpeeds();
        boolean readyToShoot = false;

        drivetrain.adjustSpeeds(outputAdjustment);
    }
}
