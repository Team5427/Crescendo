package team5427.frc.robot.util;

public final class MathUtils {
    public static double rotationsToMeters(double rotations, double diameter) {
        return rotations * Math.PI * diameter;
    }

    public static double metersToRotations(double meters, double diameter) {
        return meters / (Math.PI * diameter);
    }
}
