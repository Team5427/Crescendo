package frc.robot.util.Localization;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class LocalizationUtil {
    public static Optional<SteelTalonsVisionMeasurement> findConfidence(Optional<EstimatedRobotPose> estimate, Pose2d refPose) {
        if (estimate.isPresent()) {
            double timestamp = estimate.get().timestampSeconds;
            Pose2d pose = estimate.get().estimatedPose.toPose2d();
            Transform2d diffFromRef = pose.minus(refPose);

            double stdX = Math.abs(diffFromRef.getX()) > 0.116 ? Double.MAX_VALUE : diffFromRef.getX() * (0.3); //0.3 is brainfuck number
            double stdY = Math.abs(diffFromRef.getY()) > 0.116 ? Double.MAX_VALUE : diffFromRef.getY() * (0.3);
            Matrix<N3, N1> confidence = VecBuilder.fill(0.03, 0.03, Double.MAX_VALUE); //FIXME may need to tune
            return Optional.of(new SteelTalonsVisionMeasurement(pose, confidence, timestamp));
        } else {
            return Optional.empty();
        }
    }
}
