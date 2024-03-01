package frc.robot.util.Localization;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.Swerve.DrivetrainConstants;
import frc.robot.util.MiscUtil;

public class LocalizationUtil {
    public static Optional<SteelTalonsVisionMeasurement> findConfidence(Optional<EstimatedRobotPose> estimate, Pose2d refPose) {
        if (estimate.isPresent()) {
            double timestamp = estimate.get().timestampSeconds;
            Pose2d pose = estimate.get().estimatedPose.toPose2d();
            double diffFromRef = pose.getTranslation().minus(refPose.getTranslation()).getNorm();
            boolean trustable = 
                Math.abs(MiscUtil.targetingInformation()[0]) < 1.0 && 
                Math.abs(MiscUtil.targetingInformation()[1]) < 1.0;
                // Math.abs(MiscUtil.targetingInformation()[2]) < 4.0;
                // Math.abs(MiscUtil.targetingInformation()[3]) < Math.toRadians(30);

            double stDev = diffFromRef < DrivetrainConstants.MAX_TRANSLATION_SPEED_M_PER_LOOP || trustable ? 0.03 : Double.MAX_VALUE;
            Matrix<N3, N1> confidence = VecBuilder.fill(stDev, stDev, Double.MAX_VALUE); //FIXME may need to tune
            return Optional.of(new SteelTalonsVisionMeasurement(pose, confidence, timestamp));
        } else {
            return Optional.empty();
        }
    }
}
