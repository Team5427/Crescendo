package frc.robot.util.Localization;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class SteelTalonsVisionMeasurement {
    private Pose2d pose;
    private Matrix<N3, N1> confidence;
    private double timestamp;

    public SteelTalonsVisionMeasurement(Pose2d pose, Matrix<N3, N1> confidence, double timestamp) {
        this.pose = pose;
        this.confidence = confidence;
        this.timestamp = timestamp;
    }

    public Pose2d getPose() {
        return pose;
    }

    public Matrix<N3, N1> getConfidence() {
        return confidence;
    }

    public double getTimestamp() {
        return timestamp;
    }
}
