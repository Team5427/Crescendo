package frc.robot.subsystems.Pathfinding;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve.DrivetrainConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.util.Localization.SteelTalonsLocalization;

public class PathFinderCommand extends Command {
    SwerveDrivetrain swerve;
    PathConstraints constraints;
    Pose2d robotPose;
    Pose2d targetPose;
    SteelTalonsLocalization poseEstimator;

    public PathFinderCommand() {
        this.poseEstimator = new SteelTalonsLocalization();
        this.robotPose = poseEstimator.getPose();
        this.targetPose = new Pose2d();
        this.swerve = new SwerveDrivetrain();
        this.constraints = new PathConstraints(DrivetrainConstants.MAX_TRANSLATION_SPEED_M_S_TELEOP,
                DrivetrainConstants.MAX_ACCEL, DrivetrainConstants.MAX_ROTATION_SPEED_RAD_S_TELEOP,
                DrivetrainConstants.MAX_ACCEL);
        this.swerve = new SwerveDrivetrain();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false; // MAKE SURE THIS IS RIGHT TO PREVENT INFINITE LOOP
    }

    public PathPlannerPath getPresetPath(CommandXboxController controller, int input) {
        if (input == 1) {
            PathPlannerPath path = PathPlannerPath.fromPathFile("Half Lower To Source");
            return path;
        } else if (input == 2) {
            PathPlannerPath path = PathPlannerPath.fromPathFile("Source To Speaker");
            return path;
        } else {
            return null;
        }
    }

    public void setNewPath(PathPlannerPath presetPath) {
        AutoBuilder.pathfindThenFollowPath(presetPath, constraints, 0.3);
    }
}
