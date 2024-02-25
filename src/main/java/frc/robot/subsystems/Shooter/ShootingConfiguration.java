package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShootingConfiguration {
    private Rotation2d pivotAngle;
    private double leftRPM, rightRPM;

    public ShootingConfiguration(Rotation2d angle, double left, double right) {
        this.pivotAngle = angle;
        this.leftRPM = left;
        this.rightRPM = right;
    }

    public Rotation2d getPivotAngle() {
        return pivotAngle;
    }

    public double getLeftSpeed() {
        return this.leftRPM;
    }

    public double getRightSpeed() {
        return this.rightRPM;
    }

    public ShootingConfiguration adjustBy(Rotation2d angle, double left, double right) {
        return new ShootingConfiguration(this.pivotAngle.plus(angle), this.leftRPM + left, this.rightRPM + right);
    }

    @Override
    public String toString() {
        return "Pivot angle(deg): " + pivotAngle.getDegrees() + "\nLeft: " + leftRPM + "\nRight: " + rightRPM;
    }
}
