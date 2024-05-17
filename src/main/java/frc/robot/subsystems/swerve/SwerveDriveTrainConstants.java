package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.drivers.CANDeviceId;
import frc.robot.lib.drivers.ComplexGearRatio;
import frc.robot.lib.kinematics.ConstrainedChassisSpeeds;
import frc.robot.lib.motors.MotorConfiguration;

public final class SwerveDriveTrainConstants {
    public static final int kFrontLeftModuleIdx = 0;
    public static final int kFrontRightModuleIdx = 1;
    public static final int kRearLeftModuleIdx = 2;
    public static final int kRearRightModuleIdx = 3;

    public static final CANDeviceId[] kSteerMotorIds = { new CANDeviceId(0), new CANDeviceId(0), new CANDeviceId(0),
            new CANDeviceId(0) };
    public static final CANDeviceId[] kDriveMotorIds = { new CANDeviceId(0), new CANDeviceId(0), new CANDeviceId(0),
            new CANDeviceId(0) };
    public static final CANDeviceId[] kCancoderIds = { new CANDeviceId(0), new CANDeviceId(0), new CANDeviceId(0),
            new CANDeviceId(0) };
    public static final double[] kModuleOffsets = { 19.5, 19.5, -19.5, -19.5 }; // Module offsets from each other 0 -> 1
                                                                                // -> 2 -> 3 -> 0

    public static final double kWheelDiamterMeters = Units.inchesToMeters(3.88);

    public static final ComplexGearRatio kSDSSteerGearRatio = new ComplexGearRatio(1.0 / (150.0 / 7.0));
    public static final ComplexGearRatio kSDSL1GearRatio = new ComplexGearRatio((14.0 / 50.0), (25.0 / 19.0),
            (15.0 / 45.0));
    public static final ComplexGearRatio kSDSL2GearRatio = new ComplexGearRatio((14.0 / 50.0), (27.0 / 17.0),
            (15.0 / 45.0));
    public static final ComplexGearRatio kSDSL3GearRatio = new ComplexGearRatio((14.0 / 50.0), (28.0 / 16.0),
            (15.0 / 45.0));
    public static final ComplexGearRatio kSDSL4GearRatio = new ComplexGearRatio((16.0 / 48.0), (28.0 / 16.0),
            (15.0 / 45.0));

    public static final CANDeviceId kPidgeonID = new CANDeviceId(0); // UPDATE

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            new Translation2d(kModuleOffsets[kFrontLeftModuleIdx] / 2, kModuleOffsets[kFrontRightModuleIdx] / 2),
            new Translation2d(kModuleOffsets[kFrontRightModuleIdx] / 2, kModuleOffsets[kRearLeftModuleIdx] / 2),
            new Translation2d(kModuleOffsets[kRearLeftModuleIdx] / 2, kModuleOffsets[kFrontRightModuleIdx] / 2),
            new Translation2d(kModuleOffsets[kRearRightModuleIdx] / 2, kModuleOffsets[kRearLeftModuleIdx] / 2));

    public static final double kMaxPossibleSpeedMetersPerSecond = (5800 * (15.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0)
            * kWheelDiamterMeters * Math.PI) / (60.0);
    public static final double kMaxAcceleration = kMaxPossibleSpeedMetersPerSecond * 6;
    public static final double kCompetitionThresholdStoppingMetersPerSecond = 0.0;
    public static final double kTuningThresholdStoppingMetersPerSecond = 0.75;

    public static final double kTeleopMaxTranslationalSpeed = kMaxPossibleSpeedMetersPerSecond * 1.0;
    public static final double kTeleopMaxRotationalSpeed = Math.PI * 4;

    public static final double kMaxTranslationalSpeedPerLoop = 0.5;

    public static final ConstrainedChassisSpeeds kNormalDrivingConstraints = new ConstrainedChassisSpeeds(1.0, 1.0,
            1.0);
    public static final ConstrainedChassisSpeeds kSpeakerDrivingConstraints = new ConstrainedChassisSpeeds(0.8, 0.8,
            0.5);
    public static final ConstrainedChassisSpeeds kAmpDrivingConstraints = new ConstrainedChassisSpeeds(0.2, 0.2, 0.2);
    public static final ConstrainedChassisSpeeds kClimbDrivingConstraints = new ConstrainedChassisSpeeds(0.4, 0.4, 0.2);

}
