package frc.robot.util;

import java.util.Optional;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Shooter.ShootingConfiguration;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.util.Localization.SteelTalonsLocalization;

public class MiscUtil {
    public static final double fieldWidth = Units.feetToMeters(54);
    public static final double fieldHeight = Units.feetToMeters(27);
    public static final Pose2d speaker_Pose = new Pose2d(0, 5.5, new Rotation2d(Math.PI));
    private static final int MAX_SMAX_PERIODIC_FRAME_MS = 65534;

    public static Pose2d flip(Pose2d pose) {
        Translation2d trans = pose.getTranslation();
        Rotation2d rot = pose.getRotation();
        return new Pose2d(flip(trans), flip(rot));
    }

    public static Translation2d flip(Translation2d trans) {
        double x = trans.getX();
        x = (fieldWidth - x);
        return new Translation2d(x, trans.getY());
    }

    public static Rotation2d flip(Rotation2d rot) {
        double x = rot.getCos();
        x *= -1;
        return new Rotation2d(x, rot.getSin());
    }

    public static boolean isBlue() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            System.err.println("returning value is present");
            return alliance.get().equals(DriverStation.Alliance.Blue);
        } else {
            return false;
        }
    }

    public static Pose2d[] resetPose() { //blue, then red
        return new Pose2d[]{
            new Pose2d(),
            new Pose2d(fieldWidth, 0, new Rotation2d(Math.PI))
        };
    }

    public static void doPeriodicFrame(CANSparkMax... motor) {
        for (CANSparkMax m : motor) {
            m.setPeriodicFramePeriod(PeriodicFrame.kStatus1, MAX_SMAX_PERIODIC_FRAME_MS);
            m.setPeriodicFramePeriod(PeriodicFrame.kStatus2, MAX_SMAX_PERIODIC_FRAME_MS);
            m.setPeriodicFramePeriod(PeriodicFrame.kStatus3, MAX_SMAX_PERIODIC_FRAME_MS);
            m.setPeriodicFramePeriod(PeriodicFrame.kStatus4, MAX_SMAX_PERIODIC_FRAME_MS);
            m.setPeriodicFramePeriod(PeriodicFrame.kStatus5, MAX_SMAX_PERIODIC_FRAME_MS);
            m.setPeriodicFramePeriod(PeriodicFrame.kStatus6, MAX_SMAX_PERIODIC_FRAME_MS);
        }
    }

    public static void doPeriodicFrame(int msZero, CANSparkMax... motor) {
        for (CANSparkMax m : motor) {
            m.setPeriodicFramePeriod(PeriodicFrame.kStatus0, msZero);
        }
        doPeriodicFrame(motor);
    }

    public static void doPeriodicFrameLess(CANSparkMax... motor) {
        for (CANSparkMax m : motor) {
            m.setPeriodicFramePeriod(PeriodicFrame.kStatus3, MAX_SMAX_PERIODIC_FRAME_MS);
            m.setPeriodicFramePeriod(PeriodicFrame.kStatus4, MAX_SMAX_PERIODIC_FRAME_MS);
            m.setPeriodicFramePeriod(PeriodicFrame.kStatus5, MAX_SMAX_PERIODIC_FRAME_MS);
            m.setPeriodicFramePeriod(PeriodicFrame.kStatus6, MAX_SMAX_PERIODIC_FRAME_MS);
        }
    }

    public static InverseInterpolator<Double> getInversePoseInterpolator() {
        return new InverseInterpolator<Double>() {
            @Override
            public double inverseInterpolate(Double startValue, Double endValue, Double q) {
                double totalRange = endValue - startValue;
                if (totalRange <= 0) {
                  return 0.0;
                }
                double queryToStart = q - startValue;
                if (queryToStart <= 0) {
                  return 0.0;
                }
                return queryToStart / totalRange;
            }
        };
    }

    public static Interpolator<ShootingConfiguration> getShooterInterpolator() {
        return new Interpolator<ShootingConfiguration>() {
            @Override
            public ShootingConfiguration interpolate(ShootingConfiguration startValue, ShootingConfiguration endValue, double t) {
                Rotation2d startRotation = startValue.getPivotAngle();
                Rotation2d endRotation = endValue.getPivotAngle();
                Rotation2d dRotation = endRotation.minus(startRotation);

                double leftStartRPM = startValue.getLeftSpeed();
                double leftEndRPM = endValue.getLeftSpeed();
                double dLeftRPM = leftEndRPM - leftStartRPM;

                double rightStartRPM = startValue.getRightSpeed();
                double rightEndRPM = endValue.getRightSpeed();
                double dRightRPM = rightEndRPM - rightStartRPM;

                double clampedT = MathUtil.clamp(t, 0, 1);


                return new ShootingConfiguration(
                    dRotation.times(clampedT).plus(startRotation), 
                    dLeftRPM * clampedT + leftStartRPM,
                    dRightRPM * clampedT + rightStartRPM
                );
            }
        };
    }

    public static double[] targetingInformation() { //very much a big source of potential bugs
        SteelTalonsLocalization localization = SteelTalonsLocalization.getInstance();
        SwerveDrivetrain drivetrain = SwerveDrivetrain.getInstance();
        Transform2d transform = localization.transformFromSpeaker();
        double distance = transform.getTranslation().getNorm();
        Rotation2d angError = transform.getTranslation().getAngle().minus(localization.getPose().getRotation());
        ChassisSpeeds botSpeeds = drivetrain.getVelocityVector();
        Rotation2d velocityRot = new Rotation2d(botSpeeds.vxMetersPerSecond, botSpeeds.vyMetersPerSecond);
        double velocityMag = Math.hypot(botSpeeds.vxMetersPerSecond, botSpeeds.vyMetersPerSecond);

        double parallelSpeed = velocityMag * transform.getTranslation().getAngle().minus(velocityRot).getCos();
        double perpSpeed = velocityMag * transform.getTranslation().getAngle().minus(velocityRot).getSin();

        return new double[]{parallelSpeed, perpSpeed, distance, angError.getRadians()};
    }

}
