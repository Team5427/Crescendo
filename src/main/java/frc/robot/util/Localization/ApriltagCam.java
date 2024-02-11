package frc.robot.util.Localization;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ApriltagCam extends SubsystemBase {

    private PhotonCamera cam;
    private PhotonPoseEstimator estimator;

    public ApriltagCam(String name, Transform3d robotToCam, AprilTagFieldLayout fieldLayout) {
        cam = new PhotonCamera(name);
        estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);
        estimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
    }

    public Optional<EstimatedRobotPose> getUpdate(Pose2d refPose) {
        return estimator.update();
    }

}
