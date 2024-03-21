package frc.robot.util;

import java.util.Optional;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Shooter.ShootingConfiguration;
import frc.robot.subsystems.Swerve.DrivetrainConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.util.Localization.SteelTalonsLocalization;

public class MiscUtil {
    public static final double fieldWidth = Units.feetToMeters(54);
    public static final double fieldHeight = Units.feetToMeters(27);
    public static final Pose2d speaker_Pose = new Pose2d(0, 5.5, new Rotation2d(Math.PI));
    private static final int MAX_SMAX_PERIODIC_FRAME_MS = 65533;

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
            // System.err.println("returning value is present");
            return alliance.get().equals(DriverStation.Alliance.Blue);
        } else {
            return false;
        }
    }

    public static Pose2d[] resetPose() { //blue, then red

        double speakerX = 1.38; //1.38
        double speakerY = 5.5; //5.5

        return new Pose2d[]{
            new Pose2d(speakerX, speakerY, new Rotation2d()),
            new Pose2d(fieldWidth - speakerX, speakerY, new Rotation2d(Math.PI))
        };
    }

    public static void doPeriodicFrame(CANSparkMax... motor) {
        doPeriodicFrameLess(motor);
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
                return InverseInterpolator.forDouble().inverseInterpolate(startValue, endValue, q);
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
        Translation2d translation = localization.translationFromSpeaker();

        double distance = translation.getNorm();
        Rotation2d angError = 
            translation.getAngle(). //the angle of the line from the speaker to the bot
            // minus(new Rotation2d(0.0)). //DID A FULL ONE EIGHTYYYYYYY, CRAZYYYY, THINKING 'BOUT THE WAY I ONCE LET THE HEARTBREAK CHANGE MEEEE
            minus(localization.getPose().getRotation()); //minus the current angle of bot
        ChassisSpeeds botSpeeds = drivetrain.getVelocityVector();
        Rotation2d velocityRot = new Rotation2d(botSpeeds.vxMetersPerSecond, botSpeeds.vyMetersPerSecond);
        double velocityMag = Math.hypot(botSpeeds.vxMetersPerSecond, botSpeeds.vyMetersPerSecond);

        double parallelSpeed = velocityMag * translation.getAngle().minus(velocityRot).getSin();
        double perpSpeed = velocityMag * translation.getAngle().minus(velocityRot).getCos();

        return new double[]{MiscUtil.isBlue() ? -parallelSpeed : parallelSpeed, MiscUtil.isBlue() ? perpSpeed : -perpSpeed, distance, angError.getRadians(), translation.getAngle().getRadians()};
    }

    public static double DTrotToMeters(double rotations) {
        return rotations * Math.PI * DrivetrainConstants.WHEEL_DIAMETER_METERS;
    }

    public static double DTmetersToRot(double meters) {
        return meters / (Math.PI * DrivetrainConstants.WHEEL_DIAMETER_METERS);
    }

    public static double drivetrainSpeedMagnitude() {
        double x = SwerveDrivetrain.getInstance().getVelocityVector().vxMetersPerSecond;
        double y = SwerveDrivetrain.getInstance().getVelocityVector().vyMetersPerSecond;
        return Math.hypot(x, y);
    }

}
