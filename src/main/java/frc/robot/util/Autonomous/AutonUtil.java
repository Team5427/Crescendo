package frc.robot.util.Autonomous;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter.HardCodeShot;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Swerve.DrivetrainConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.managing.SubsystemManager;
import frc.robot.util.MiscUtil;
import frc.robot.util.Localization.SteelTalonsLocalization;

public class AutonUtil extends SubsystemBase implements PathSwitcher, CommandSwitcher, Conditionals {

    private final double CHOREO_TRANSLATION_KP = 2.0; // 28.0
    private final double CHOREO_ROTATION_KP = 7.0; // 3.5

    // public static Command // ALL PATHING COMMAND
    // bottomSidePreloadToFirst,
    // bottomSideFirstToShoot,
    // bottomSideFirstToSecond,
    // bottomSideShootToSecond,
    // bottomSideShootToThird,
    // bottomSideSecondToThird,
    // bottomSideThirdToSecond,
    // bottomSideSecondToShoot,
    // bottomSideThirdToShoot,
    // bottomSideFirstToThird,
    // bottomSideShootToMid; // set to remove

    // public static SequentialCommandGroup shootPreloadAndFirstIntake,
    // shootFirst,
    // shootSecond,
    // shootThird,
    // secondIntake,
    // thirdIntake,
    // missFirstToThird,
    // missFirstToSecond,
    // missSecondToThird,
    // missThirdToSecond,
    // adaptiveBottomSideFourNote; // set to remove

    public AutonUtil() {
        AutoBuilder.configureHolonomic(
                SteelTalonsLocalization.getInstance()::getOdometryPose,
                SteelTalonsLocalization.getInstance()::resetOdometryPose,
                SwerveDrivetrain.getInstance()::getVelocityVector,
                SwerveDrivetrain.getInstance()::setSpeedsAuton,
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(CHOREO_TRANSLATION_KP, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(CHOREO_ROTATION_KP, 0.0, 0.0), // Rotation PID constants
                        DrivetrainConstants.MAX_PHYSICAL_SPEED_M_S * 0.75, // Max module speed, in m/s
                        Math.hypot(DrivetrainConstants.TRACKWIDTH, DrivetrainConstants.WHEELBASE) / 2.0, // Drive base
                                                                                                         // radius in
                                                                                                         // meters.
                                                                                                         // Distance
                                                                                                         // from robot
                                                                                                         // center to
                                                                                                         // furthest
                                                                                                         // module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    return !MiscUtil.isBlue();
                },
                SwerveDrivetrain.getInstance());

        addPaths();
        addCommands();

        // PPHolonomicDriveController.setRotationTargetOverride(() -> {
        // if (RobotContainer.getNoteCam().noteInRange()) {
        // return Optional.of(
        // SteelTalonsLocalization.getInstance().getPose().getRotation().plus(
        // RobotContainer.getNoteCam().targetXRot()));
        // } else {
        // return Optional.empty();
        // }
        // });
    }

    public void addPaths() {
        PathSwitcher.addPath("bottomSidePreloadToFirst",
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-1.1")));
        PathSwitcher.addPath("bottomSideFirstToShoot",
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-1.2")));
        PathSwitcher.addPath("bottomSideShootToThird",
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-1.3")));
        PathSwitcher.addPath("bottomSideThirdToShoot",
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-1.4")));
        PathSwitcher.addPath("bottomSideShootToSecond",
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-1.5")));
        PathSwitcher.addPath("bottomSideSecondToShoot",
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-1.6")));
        PathSwitcher.addPath("bottomSideShootToMid",
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-1.7")));
        PathSwitcher.addPath("bottomSideFirstToSecond",
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-2.1")));
        PathSwitcher.addPath("bottomSideSecondToThird",
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-2.2")));
        PathSwitcher.addPath("bottomSideThirdToSecond",
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-3.2")));
        PathSwitcher.addPath("bottomSideFirstToThird",
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("B-3.1")));

    }

    public void addCommands() {
        CommandSwitcher.addCommand("shootPreloadAndFirstIntake", new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        PathSwitcher.getPath("bottomSidePreloadToFirst"),
                        new SequentialCommandGroup(
                                new HardCodeShot(ShooterConstants.FIRST_AUTON_SHOT_CONFIGURATION),
                                new WaitCommand(0.5),
                                SubsystemManager.getComplexIntakeCommand()))));

        CommandSwitcher.addCommand("shootFirst", new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                SubsystemManager.getComplexIntakeCommand(),
                                SubsystemManager.getTargetingCommand()),
                        PathSwitcher.getPath("bottomSideFirstToShoot"))));

        CommandSwitcher.addCommand("shootSecond", new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                SubsystemManager.getComplexIntakeCommand(),
                                SubsystemManager.getTargetingCommand()),
                        PathSwitcher.getPath("bottomSideSecondToShoot"))));

        CommandSwitcher.addCommand("shootThird", new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                SubsystemManager.getComplexIntakeCommand(),
                                SubsystemManager.getTargetingCommand()),
                        PathSwitcher.getPath("bottomSideThirdToShoot"))));

        CommandSwitcher.addCommand("secondIntake", new SequentialCommandGroup(

                new ParallelDeadlineGroup(
                        PathSwitcher.getPath("bottomSideShootToSecond"),
                        new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                SubsystemManager.getComplexIntakeCommand()))));

        CommandSwitcher.addCommand("thirdIntake", new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        PathSwitcher.getPath("bottomSideShootToThird"),
                        new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                SubsystemManager.getComplexIntakeCommand()))));

        CommandSwitcher.addCommand("missFirstToSecond", new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        PathSwitcher.getPath("bottomSideFirstToSecond"),
                        new SequentialCommandGroup(
                                new WaitCommand(0.1),
                                SubsystemManager.getComplexIntakeCommand()))));

        CommandSwitcher.addCommand("missFirstToThird", new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        PathSwitcher.getPath("bottomSideFirstToThird"),
                        new SequentialCommandGroup(
                                new WaitCommand(0.1),
                                SubsystemManager.getComplexIntakeCommand()))));

        CommandSwitcher.addCommand("missSecondToThird", new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        PathSwitcher.getPath("bottomSideSecondToThird"),
                        new SequentialCommandGroup(
                                new WaitCommand(0.1),
                                SubsystemManager.getComplexIntakeCommand()))));

        CommandSwitcher.addCommand("missThirdToSecond", new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        PathSwitcher.getPath("bottomSideThirdToSecond"),
                        new SequentialCommandGroup(
                                new WaitCommand(0.1),
                                SubsystemManager.getComplexIntakeCommand()))));

        CommandSwitcher.addCommand("adaptiveBottomSideFourNote", new SequentialCommandGroup(
                CommandSwitcher.getCommand("shootPreloadAndFirstIntake"),
                new ConditionalCommand(
                        CommandSwitcher.getCommand("shootFirst").andThen(CommandSwitcher.getCommand("thirdIntake")),
                        CommandSwitcher.getCommand("missFirstToThird"),
                        Conditionals.hasNote),
                new ConditionalCommand(
                        CommandSwitcher.getCommand("shootThird").andThen(CommandSwitcher.getCommand("secondIntake")),
                        PathSwitcher.getPath("missThirdToSecond"),
                        Conditionals.hasNote),
                new ConditionalCommand(
                        CommandSwitcher.getCommand("shootSecond"),
                        PathSwitcher.getPath("missSecondToThird"),
                        Conditionals.hasNote),
                PathSwitcher.getPath("bottomSideShootToMid"))); // TODO: add the methods into another interface
    }
}
