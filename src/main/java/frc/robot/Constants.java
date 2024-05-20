package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.drivers.CANDeviceId;
import frc.robot.lib.drivers.ComplexGearRatio;
import frc.robot.lib.kinematics.ConstrainedChassisSpeeds;
import frc.robot.lib.motors.MotorConfiguration;
import frc.robot.lib.motors.MotorConfiguration.IdleState;
import frc.robot.lib.motors.MotorConfiguration.MotorMode;

public final class Constants {
        public final class ShooterConstants { // Add to Constants file as a subclass
                public static final Rotation2d kStow = new Rotation2d(30); // UPDATE
                public static final Rotation2d kHandoff = new Rotation2d(10); // UPDATE
                public static final Rotation2d kDefaultShooting = new Rotation2d(80); // UPDATE
                public static final Rotation2d kShootAmp = new Rotation2d(90); // UPDATE

                public static final double kFlywheelTolerance = 0.01; // UPDATE

                public static final Rotation2d kStowAmp = new Rotation2d(0); // UPDATE
                public static final Rotation2d kIdleAmp = new Rotation2d(20); // UPDATE
                public static final Rotation2d kUseAmp = new Rotation2d(100); // UPDATE

                public static final CANDeviceId kTopMotorID = new CANDeviceId(0); // UPDATE
                public static final CANDeviceId kBottomMotorID = new CANDeviceId(0); // UPDATE
                public static final CANDeviceId kAmpMotorID = new CANDeviceId(0); // UPDATE
                public static final CANDeviceId kFeederMotorID = new CANDeviceId(0); // UPDATE
                public static final CANDeviceId kPivotLeaderMotorID = new CANDeviceId(0); // UPDATE
                public static final CANDeviceId kPivotFollowerMotorID = new CANDeviceId(0); // UPDATE

                public static final int kEarlyBeamBrakeID = 0; // UPDATE
                public static final int kLateBeamBrakeID = 0; // UPDATE

                public static final ComplexGearRatio kAmpMotorGearing = new ComplexGearRatio(); // UPDATE
                public static final ComplexGearRatio kPivotFollowerMotorGearing = new ComplexGearRatio(); // UPDATE
                public static final ComplexGearRatio kPivotLeaderMotorGearing = new ComplexGearRatio(); // UPDATE
                public static final ComplexGearRatio kTopMotorGearing = new ComplexGearRatio(); // UPDATE
                public static final ComplexGearRatio kBottomMotorGearing = new ComplexGearRatio(); // UPDATE
                public static final ComplexGearRatio kFeederMotorGearing = new ComplexGearRatio(); // UPDATE

                public static MotorConfiguration kAmpMotorConfiguration = new MotorConfiguration();
                public static MotorConfiguration kPivotLeaderMotorConfiguration = new MotorConfiguration();
                public static MotorConfiguration kPivotFollowerMotorConfiguration = new MotorConfiguration();
                public static MotorConfiguration kTopMotorConfiguration = new MotorConfiguration();
                public static MotorConfiguration kBottomMotorConfiguration = new MotorConfiguration();
                public static MotorConfiguration kFeederMotorConfiguration = new MotorConfiguration();

                static {
                        kAmpMotorConfiguration.gearRatio = kAmpMotorGearing;
                        kAmpMotorConfiguration.currentLimit = 15; // UPDATE
                        kAmpMotorConfiguration.idleState = IdleState.kBrake;
                        kAmpMotorConfiguration.isInverted = false; // UPDATE
                        kAmpMotorConfiguration.mode = MotorMode.kServo;
                        kAmpMotorConfiguration.unitConversionRatio = 1.0; // UPDATE
                        kAmpMotorConfiguration.kP = 0.0; // UPDATE
                        kAmpMotorConfiguration.kFF = 0.0; // UPDATE

                        kTopMotorConfiguration.gearRatio = kTopMotorGearing;
                        kTopMotorConfiguration.currentLimit = 30; // UPDATE
                        kTopMotorConfiguration.mode = MotorMode.kFlywheel;
                        kTopMotorConfiguration.isInverted = false; // UPDATE
                        kTopMotorConfiguration.idleState = IdleState.kCoast;
                        kTopMotorConfiguration.finalDiameterMeters = 1.0; // UPDATE
                        kTopMotorConfiguration.kFF = 1.0; // UPDATE
                        kTopMotorConfiguration.kA = 0.0; // UPDATE
                        kTopMotorConfiguration.kS = 1.0; // UPDATE
                        kTopMotorConfiguration.kV = 0.0; // UPDATE

                        kBottomMotorConfiguration.gearRatio = kTopMotorGearing;
                        kBottomMotorConfiguration.currentLimit = 30; // UPDATE
                        kBottomMotorConfiguration.mode = MotorMode.kFlywheel;
                        kBottomMotorConfiguration.isInverted = true; // UPDATE
                        kBottomMotorConfiguration.idleState = IdleState.kCoast;
                        kBottomMotorConfiguration.finalDiameterMeters = 1.0; // UPDATE
                        kBottomMotorConfiguration.kFF = 1.0; // UPDATE
                        kBottomMotorConfiguration.kA = 0.0; // UPDATE
                        kBottomMotorConfiguration.kS = 1.0; // UPDATE
                        kBottomMotorConfiguration.kV = 0.0; // UPDATE

                        kPivotLeaderMotorConfiguration.currentLimit = 30; // UPDATE
                        kPivotLeaderMotorConfiguration.gearRatio = kPivotLeaderMotorGearing;
                        kPivotLeaderMotorConfiguration.isInverted = false; // UPDATE
                        kPivotLeaderMotorConfiguration.idleState = IdleState.kBrake;
                        kPivotLeaderMotorConfiguration.mode = MotorMode.kServo; // UPDATE
                        kPivotLeaderMotorConfiguration.kFF = 1.0; // UPDATE

                        kPivotFollowerMotorConfiguration.currentLimit = 30; // UPDATE
                        kPivotFollowerMotorConfiguration.gearRatio = kPivotLeaderMotorGearing;
                        kPivotFollowerMotorConfiguration.isInverted = false; // UPDATE
                        kPivotFollowerMotorConfiguration.idleState = IdleState.kBrake;
                        kPivotFollowerMotorConfiguration.mode = MotorMode.kServo; // UPDATE
                        kPivotFollowerMotorConfiguration.kFF = 1.0; // UPDATE

                        kFeederMotorConfiguration.currentLimit = 30; // UPDATE
                        kFeederMotorConfiguration.gearRatio = kFeederMotorGearing;
                        kFeederMotorConfiguration.isInverted = false; // UPDATE
                        kFeederMotorConfiguration.mode = MotorMode.kLinear;
                        kFeederMotorConfiguration.idleState = IdleState.kBrake;
                        kFeederMotorConfiguration.kFF = 1.0;
                        kFeederMotorConfiguration.maxAcceleration = 1.0;

                }
        }

        public final class DriveTrainConstants {
                public static final int kFrontLeftModuleIdx = 0;
                public static final int kFrontRightModuleIdx = 1;
                public static final int kRearLeftModuleIdx = 2;
                public static final int kRearRightModuleIdx = 3;

                public static final CANDeviceId[] kSteerMotorIds = { new CANDeviceId(0), new CANDeviceId(0),
                                new CANDeviceId(0),
                                new CANDeviceId(0) };
                public static final CANDeviceId[] kDriveMotorIds = { new CANDeviceId(0), new CANDeviceId(0),
                                new CANDeviceId(0),
                                new CANDeviceId(0) };
                public static final CANDeviceId[] kCancoderIds = { new CANDeviceId(0), new CANDeviceId(0),
                                new CANDeviceId(0),
                                new CANDeviceId(0) };
                public static final double[] kModuleOffsets = { 19.5, 19.5, -19.5, -19.5 }; // Module offsets from each
                                                                                            // other 0
                                                                                            // -> 1
                                                                                            // -> 2 -> 3 -> 0

                public static final double kWheelDiamterMeters = Units.inchesToMeters(3.88);

                public static final ComplexGearRatio kSDSSteerGearRatio = new ComplexGearRatio(1.0 / (150.0 / 7.0));
                public static final ComplexGearRatio kSDSL1GearRatio = new ComplexGearRatio((14.0 / 50.0),
                                (25.0 / 19.0),
                                (15.0 / 45.0));
                public static final ComplexGearRatio kSDSL2GearRatio = new ComplexGearRatio((14.0 / 50.0),
                                (27.0 / 17.0),
                                (15.0 / 45.0));
                public static final ComplexGearRatio kSDSL3GearRatio = new ComplexGearRatio((14.0 / 50.0),
                                (28.0 / 16.0),
                                (15.0 / 45.0));
                public static final ComplexGearRatio kSDSL4GearRatio = new ComplexGearRatio((16.0 / 48.0),
                                (28.0 / 16.0),
                                (15.0 / 45.0));

                public static final CANDeviceId kPidgeonID = new CANDeviceId(0); // UPDATE

                public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
                                new Translation2d(kModuleOffsets[kFrontLeftModuleIdx] / 2,
                                                kModuleOffsets[kFrontRightModuleIdx] / 2),
                                new Translation2d(kModuleOffsets[kFrontRightModuleIdx] / 2,
                                                kModuleOffsets[kRearLeftModuleIdx] / 2),
                                new Translation2d(kModuleOffsets[kRearLeftModuleIdx] / 2,
                                                kModuleOffsets[kFrontRightModuleIdx] / 2),
                                new Translation2d(kModuleOffsets[kRearRightModuleIdx] / 2,
                                                kModuleOffsets[kRearLeftModuleIdx] / 2));

                public static final double kMaxPossibleSpeedMetersPerSecond = (5800 * (15.0 / 50.0) * (28.0 / 16.0)
                                * (15.0 / 45.0)
                                * kWheelDiamterMeters * Math.PI) / (60.0);
                public static final double kMaxAcceleration = kMaxPossibleSpeedMetersPerSecond * 6;
                public static final double kCompetitionThresholdStoppingMetersPerSecond = 0.0;
                public static final double kTuningThresholdStoppingMetersPerSecond = 0.75;

                public static final double kTeleopMaxTranslationalSpeed = kMaxPossibleSpeedMetersPerSecond * 1.0;
                public static final double kTeleopMaxRotationalSpeed = Math.PI * 4;

                public static final double kMaxTranslationalSpeedPerLoop = 0.5;

                public static final ConstrainedChassisSpeeds kNormalDrivingConstraints = new ConstrainedChassisSpeeds(
                                1.0, 1.0,
                                1.0);
                public static final ConstrainedChassisSpeeds kSpeakerDrivingConstraints = new ConstrainedChassisSpeeds(
                                0.8, 0.8,
                                0.5);
                public static final ConstrainedChassisSpeeds kAmpDrivingConstraints = new ConstrainedChassisSpeeds(0.2,
                                0.2,
                                0.2);
                public static final ConstrainedChassisSpeeds kClimbDrivingConstraints = new ConstrainedChassisSpeeds(
                                0.4, 0.4,
                                0.2);
                public static final MotorConfiguration[] kDriveMotorConfigurations = new MotorConfiguration[DriveTrainConstants.kDriveMotorIds.length];

                public static final MotorConfiguration[] kSteerMotorConfigurations = new MotorConfiguration[DriveTrainConstants.kSteerMotorIds.length];

                public static final CANcoderConfiguration[] kCanCoderConfigurations = new CANcoderConfiguration[DriveTrainConstants.kSteerMotorIds.length];

                public static final double kEncoderOffset = 0.0;

                static {
                        for (MotorConfiguration config : kDriveMotorConfigurations) {
                                config.currentLimit = 70;
                                // TODO
                        }

                        for (MotorConfiguration config : kSteerMotorConfigurations) {
                                config.currentLimit = 40;
                                // TODO
                        }
                        for (CANcoderConfiguration config : kCanCoderConfigurations) {
                                config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
                                config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
                                config.MagnetSensor.MagnetOffset = kEncoderOffset;
                        }
                }
        }

        public final class IntakeConstants {
                public static final Rotation2d kIntake = new Rotation2d(Units.degreesToRadians(200)); // UPDATE
                public static final Rotation2d kHandoff = new Rotation2d(Units.degreesToRadians(30)); // UPDATE
                public static final Rotation2d kStow = new Rotation2d(Units.degreesToRadians(80)); // UPDATE
                public static final double kIdleSpeed = 0.1; // UPDATE
                public static final double kIntakeSpeed = 0.8; // UPDATE
                public static final CANDeviceId kPivotMotorID = new CANDeviceId(0); // UPDATE
                public static final CANDeviceId kIntakeMotorID = new CANDeviceId(0, "*");
                public static final int kBeamBreakPort = 0;

                public static MotorConfiguration kPivotMotorConfiguration = new MotorConfiguration();
                public static MotorConfiguration kintakeMotorConfiguration = new MotorConfiguration();

                static { // UPDATE
                        kPivotMotorConfiguration.currentLimit = 30;
                        kPivotMotorConfiguration.gearRatio = new ComplexGearRatio(1, 2, 3);
                }
        }

}
