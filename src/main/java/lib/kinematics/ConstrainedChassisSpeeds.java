package lib.kinematics;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ConstrainedChassisSpeeds {

    private ChassisSpeeds constrainedSpeeds;

    private Optional<Double> xConstraint;
    private Optional<Double> yConstraint;
    private Optional<Rotation2d> rotationConstraint;

    public ConstrainedChassisSpeeds() {
        this(new ChassisSpeeds());
    }

    public ConstrainedChassisSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        this(new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond));
    }

    public ConstrainedChassisSpeeds(ChassisSpeeds speeds) {
        constrainedSpeeds = speeds;

        xConstraint = Optional.empty();
        yConstraint = Optional.empty();
        rotationConstraint = Optional.empty();
    }

    public void resetChassisSpeeds(ChassisSpeeds speeds) {
        constrainedSpeeds = speeds;
    }

    public ChassisSpeeds getCalculatedSpeeds() {
        ChassisSpeeds calculatedSpeeds = new ChassisSpeeds(
            xConstraint.isPresent() ? xConstraint.get(): constrainedSpeeds.vxMetersPerSecond,
            yConstraint.isPresent() ? yConstraint.get(): constrainedSpeeds.vyMetersPerSecond,
            rotationConstraint.isPresent() ? rotationConstraint.get().getRadians(): constrainedSpeeds.omegaRadiansPerSecond
        );
        return calculatedSpeeds;
    }

    public void removeXConstraint() {
        xConstraint = Optional.empty();
    }

    public void removeYConstraint() {
        yConstraint = Optional.empty();
    }

    public void removeRotationConstraint() {
        rotationConstraint = Optional.empty();
    }

    public void removeAllConstraints() {
        removeXConstraint();
        removeYConstraint();
        removeRotationConstraint();
    }

    public void setXConstraint(double vx) {
        xConstraint = Optional.of(vx);
    }

    public void setYConstraint(double vy) {
        yConstraint = Optional.of(vy);
    }

    public void setRotationConstraints(double radians) {
        rotationConstraint = Optional.of(new Rotation2d(radians));
    }

    public void setRotationConstraints(Rotation2d radians) {
        rotationConstraint = Optional.of(radians);
    }

    public void setGlobalConstraints(ChassisSpeeds constraints) {
        setXConstraint(constraints.vxMetersPerSecond);
        setYConstraint(constraints.vyMetersPerSecond);
        setRotationConstraints(constraints.omegaRadiansPerSecond);
    }

    public void setGlobalConstraints(double vx, double vy, double radians) {
        setXConstraint(vx);
        setYConstraint(vy);
        setRotationConstraints(radians);
    }

    public void setGlobalConstraints(double vx, double vy, Rotation2d radians) {
        setXConstraint(vx);
        setYConstraint(vy);
        setRotationConstraints(radians);
    }

    public double getXConstraint() {
        return xConstraint.get();
    }

    public double getYConstraint() {
        return yConstraint.get();
    }

    public Rotation2d getRotationConstraintAsRotation2d() {
        return rotationConstraint.get();
    }

    public double getRotationConstraintAsDouble() {
        return rotationConstraint.get().getRadians();
    }
    
}
