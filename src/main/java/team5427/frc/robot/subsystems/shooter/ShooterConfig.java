package team5427.frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterConfig { // replace
    public double topRPM;
    public double bottomRPM;
    public Rotation2d angle;

    public ShooterConfig(double topRPM, double bottomRPM, Rotation2d angle) {
        this.angle = angle;
        this.topRPM = topRPM;
        this.bottomRPM = bottomRPM;
    }

    @Override
    public String toString() {
        return "Pivot angle(deg): " + angle.getDegrees() + "\nTop: " + topRPM + "\nBottom: " + bottomRPM;
    }
}
