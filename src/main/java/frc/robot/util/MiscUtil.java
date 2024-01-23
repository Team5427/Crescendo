package frc.robot.util;

import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MiscUtil {
    private static final double fieldWidth = Units.feetToMeters(54);
    private static final double fieldHeight = Units.feetToMeters(27);
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

}
