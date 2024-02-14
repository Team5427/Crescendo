package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.*;

public class ShooterAutoAlign {
    Pose3d speakerPose; // WILL NEED TO BE IN CONSTANTS AND UPDATED
    Pose3d robotPose3d;
    Pose3d shooterPose3d;

    public ShooterAutoAlign(Pose3d robotPose3d, Pose3d shooterPose3d) {
        this.robotPose3d = robotPose3d;
        Rotation3d rotation = new Rotation3d(0, 0, 0);
        this.speakerPose = new Pose3d(0, 2, 4, rotation);
        this.shooterPose3d = shooterPose3d;
    }

    public double calculateRobotAngleMovement() {
        double dy = robotPose3d.getY() - speakerPose.getY();
        double dx = robotPose3d.getX() - speakerPose.getX();
        double dys = dy * dy;
        double dxs = dx * dx;
        double u = Math.sqrt(dxs + dys);
        double omega = 1 / Math.acos(u / dx);
        double b = 90 - omega;
        double theta_error = 2 * Math.PI - (b + Units.degreesToRadians(90) + robotPose3d.getRotation().getAngle());
        return theta_error;
    }

    public double calculateShooterAngleMovement(){
        double dz = 1.5 - speakerPose.getZ() + 0.0; // height of shooter, must change and update to constants and Height of april tag-same thing.
        double dx = robotPose3d.getX() - speakerPose.getX();
        double dzs = dz * dz;
        double dxs = dx * dx;
        double u = Math.sqrt(dxs + dzs);
        double omega = 1 / Math.acos(u / dx);
        double b = 90 - omega;
        double theta_error = 2 * Math.PI - (b + Units.degreesToRadians(90) + robotPose3d.getRotation().getAngle());
        return theta_error;
    }
}