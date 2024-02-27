package frc.robot.util.Localization;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.Swerve.DrivetrainConstants;

public class LocalizationUtil {
    public static Optional<SteelTalonsVisionMeasurement> findConfidence(Optional<EstimatedRobotPose> estimate, Pose2d refPose) {
        if (estimate.isPresent()) {
            double timestamp = estimate.get().timestampSeconds;
            Pose2d pose = estimate.get().estimatedPose.toPose2d();
            double diffFromRef = pose.getTranslation().minus(refPose.getTranslation()).getNorm();

            double stDev = diffFromRef < DrivetrainConstants.MAX_TRANSLATION_SPEED_M_PER_LOOP ? 0.03 : Double.MAX_VALUE;
            Matrix<N3, N1> confidence = VecBuilder.fill(stDev, stDev, Double.MAX_VALUE); //FIXME may need to tune
            return Optional.of(new SteelTalonsVisionMeasurement(pose, confidence, timestamp));
        } else {
            return Optional.empty();
        }
    }
}
