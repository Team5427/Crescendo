// package frc.robot.subsystems.Pathfinding;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.subsystems.Swerve.DrivetrainConstants;
// import frc.robot.subsystems.Swerve.SwerveDrivetrain;
// import frc.robot.util.Localization.SteelTalonsLocalization;

// public class PathFinder extends SubsystemBase {
// Pose2d targetPose; // Putting a dummy value for now
// PathConstraints constraints;
// SwerveDrivetrain swerve;
// Pose2d robotPose;
// PathFinder instance;

// public PathFinder() {
// instance = this;
// SteelTalonsLocalization poseEstimator = new SteelTalonsLocalization();
// this.robotPose = poseEstimator.getPose();
// this.targetPose = new Pose2d();
// this.swerve = new SwerveDrivetrain();
// this.constraints = new
// PathConstraints(DrivetrainConstants.MAX_TRANSLATION_SPEED_M_S_TELEOP,
// DrivetrainConstants.MAX_ACCEL,
// DrivetrainConstants.MAX_ROTATION_SPEED_RAD_S_TELEOP,
// DrivetrainConstants.MAX_ACCEL);
// }

// public PathFinder getInstance() {
// return instance;
// }

// public void setNewPath(PathPlannerPath presetPath) {
// AutoBuilder.pathfindThenFollowPath(presetPath, constraints, 3.0);
// }

// public PathPlannerPath getPresetPath(CommandXboxController controller) {
// if (controller.a().getAsBoolean() == true) { // change these values to make
// sure that they dont conflict with
// // any other buttons
// PathPlannerPath path = PathPlannerPath.fromPathFile("Half Lower To Source");
// return path;
// } else if (controller.b().getAsBoolean() == true) {
// PathPlannerPath path = PathPlannerPath.fromPathFile("Source To Speaker");
// return path;
// } else {
// return null;
// }
// }

// public Command getPathCommand(CommandXboxController xboxController) {
// return new PathFinderCommand();
// }

// }
