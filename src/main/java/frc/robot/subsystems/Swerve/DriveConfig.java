package frc.robot.subsystems.Swerve;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;

public class DriveConfig {
    private Optional<Rotation2d> angleLock = Optional.empty();
    private double speedScalar = 1.0;
    private double deadZone = 0.0;
    private boolean fieldOp = true;

    public DriveConfig(Optional<Rotation2d> angleLock, double speedScalar, double deadZone, boolean fieldOp) {
        this.angleLock = angleLock;
        this.speedScalar = speedScalar;
        this.fieldOp = fieldOp;
        this.deadZone = deadZone;
    }

    public void setAngleLock(Optional<Rotation2d> angleLock) {
        this.angleLock = angleLock;
    }

    public void setScalar(double speedScalar) {
        this.speedScalar = speedScalar;
    }

    public void setDeadzone(double deadZone) {
        this.deadZone = deadZone;
    }

    public void setFieldOp(boolean fieldOp) {
        this.fieldOp = fieldOp;
    }

    public Optional<Rotation2d> getAngleLock() {
        return angleLock;
    }

    public double getSpeedScalar() {
        return speedScalar;
    }

    public double getDeadZone() {
        return deadZone;
    }

    public boolean getFieldOp() {
        return fieldOp;
    }

    
}
