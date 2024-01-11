package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.DrivetrainConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.util.Localization.SteelTalonsLocalization;

public class AutonUtil extends SubsystemBase {
    public AutonUtil() {
        AutoBuilder.configureHolonomic(
            SteelTalonsLocalization.getInstance()::getPose, 
            SteelTalonsLocalization.getInstance()::resetPose, 
            SwerveDrivetrain.getInstance()::getVelocityVector, 
            SwerveDrivetrain.getInstance()::setSpeedsAuton, 
            new HolonomicPathFollowerConfig(
                DrivetrainConstants.MAX_PHYSICAL_SPEED_M_S, 
                Math.hypot(DrivetrainConstants.TRACKWIDTH, DrivetrainConstants.WHEELBASE), 
                new ReplanningConfig()
            ), 
            () -> {
                return MiscUtil.isBlue();
            },
            SwerveDrivetrain.getInstance()
        );

    }
}
