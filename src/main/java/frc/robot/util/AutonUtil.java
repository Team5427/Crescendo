package frc.robot.util;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.HardCodeShot;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Swerve.DrivetrainConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.managing.SubsystemManager;
import frc.robot.util.Localization.SteelTalonsLocalization;

public class AutonUtil extends SubsystemBase {

    private final double CHOREO_TRANSLATION_KP = 2.0; // 28.0
    private final double CHOREO_ROTATION_KP = 7.0; // 3.5

    public static Command 
        bottomSidePreloadToFirst,
        bottomSideFirstToShoot,
        bottomSideFirstToSecond,
        bottomSideShootToSecond,
        bottomSideShootToThird,
        bottomSideSecondToThird,
        bottomSideThirdToSecond,
        bottomSideSecondToShoot,
        bottomSideThirdToShoot;

    public static SequentialCommandGroup 
        shootPreloadAndFirstIntake, 
        shootFirstAndThirdIntake,
        shootThirdAndSecondIntake;

    public AutonUtil() {
        AutoBuilder.configureHolonomic(
            SteelTalonsLocalization.getInstance()::getOdometryPose, 
            SteelTalonsLocalization.getInstance()::resetOdometryPose, 
            SwerveDrivetrain.getInstance()::getVelocityVector, 
            SwerveDrivetrain.getInstance()::setSpeedsAuton, 
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(CHOREO_TRANSLATION_KP, 0.0, 0.0), // Translation PID constants
                new PIDConstants(CHOREO_ROTATION_KP, 0.0, 0.0), // Rotation PID constants
                DrivetrainConstants.MAX_PHYSICAL_SPEED_M_S * 0.75, // Max module speed, in m/s
                Math.hypot(DrivetrainConstants.TRACKWIDTH, DrivetrainConstants.WHEELBASE)/2.0, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ), 
            () -> {
                return !MiscUtil.isBlue();
            },
            SwerveDrivetrain.getInstance()
        );

        bottomSidePreloadToFirst = AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-1.1"));
        bottomSideFirstToShoot = AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-1.2"));
        bottomSideShootToThird = AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-1.3"));
        bottomSideThirdToShoot = AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-1.4"));
        bottomSideShootToSecond = AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-1.5"));
        bottomSideSecondToShoot = AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-1.6"));
        bottomSideFirstToSecond = AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-2.1"));
        bottomSideSecondToThird = AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-2.2"));

        shootPreloadAndFirstIntake = new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new HardCodeShot(ShooterConstants.FIRST_AUTON_SHOT_CONFIGURATION),
                    new WaitCommand(0.5),
                    SubsystemManager.getComplexIntakeCommand()
                ),
                bottomSidePreloadToFirst
            )
        );



        // PPHolonomicDriveController.setRotationTargetOverride(() -> {
        //     if (RobotContainer.getNoteCam().noteInRange()) {
        //         return Optional.of(
        //             SteelTalonsLocalization.getInstance().getPose().getRotation().plus(
        //             RobotContainer.getNoteCam().targetXRot()));
        //     } else {
        //         return Optional.empty();
        //     }
        // });
    }
}
