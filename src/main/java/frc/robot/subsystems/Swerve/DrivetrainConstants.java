package frc.robot.subsystems.Swerve;

import java.util.Optional;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.MiscUtil;
import frc.robot.util.STSmaxConfig;

public class DrivetrainConstants {
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.88); // 3.88
    public static final double TRACKWIDTH = Units.inchesToMeters(19.5);
    public static final double WHEELBASE = Units.inchesToMeters(19.5);

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEELBASE / 2, TRACKWIDTH / 2),
            new Translation2d(WHEELBASE / 2, -TRACKWIDTH / 2),
            new Translation2d(-WHEELBASE / 2, TRACKWIDTH / 2),
            new Translation2d(-WHEELBASE / 2, -TRACKWIDTH / 2));

    public static final int PIGEON_CAN_ID = 16;

    public static final double MAX_PHYSICAL_SPEED_M_S = (5800 * (15.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0)
            * WHEEL_DIAMETER_METERS * Math.PI) / (60.0);
    public static final double MAX_ACCEL = MAX_PHYSICAL_SPEED_M_S * 6; // BEING USED IN PATH FINDER
    public static final double THRESHOLD_STOPPING_M_S_COMPETITION = 0.0;
    public static final double THRESHOLD_STOPPING_M_S_TUNING = 0.75;

    public static final DriveConfig DEFAULT_DRIVE_CONFIG = new DriveConfig(
        Optional.empty(), 
        1.0, 
        0.0, 
        true
    );

    public static final DriveConfig AMP_DRIVE_CONFIG = new DriveConfig(
        Optional.of(new Rotation2d(-Math.PI/2)), 
        0.5, 
        0.25, 
        true
    );

    public static final DriveConfig SHUTTLE_DRIVE_CONFIG = new DriveConfig(
        Optional.of(new Rotation2d(-Math.PI / 5.9)), 
        1.0, 
        0.0, 
        true
    );

    public static final double MAX_TRANSLATION_SPEED_M_S_TELEOP = MAX_PHYSICAL_SPEED_M_S * 1.0;
    public static final double MAX_ROTATION_SPEED_RAD_S_TELEOP = Math.PI * 4;

    public static final double MAX_TRANSLATION_SPEED_M_PER_LOOP = 0.5;

    public static final int FRONT_LEFT_CANCODER_ID = 12;
    public static final int FRONT_RIGHT_CANCODER_ID = 13;
    public static final int BACK_LEFT_CANCODER_ID = 14;
    public static final int BACK_RIGHT_CANCODER_ID = 15;

    public static final double FRONT_LEFT_OFFSET = 0.48;
    public static final double FRONT_RIGHT_OFFSET = 0.255;
    public static final double BACK_LEFT_OFFSET = -0.38;
    public static final double BACK_RIGHT_OFFSET = 0.278;

    public static final TalonFXConfiguration FRONT_LEFT_DRIVE = new TalonFXConfiguration();
    public static final TalonFXConfiguration FRONT_RIGHT_DRIVE = new TalonFXConfiguration();
    public static final TalonFXConfiguration BACK_LEFT_DRIVE = new TalonFXConfiguration();
    public static final TalonFXConfiguration BACK_RIGHT_DRIVE = new TalonFXConfiguration();

    public static final int FRONT_LEFT_DRIVE_ID = 5;
    public static final int FRONT_RIGHT_DRIVE_ID = 3;
    public static final int BACK_LEFT_DRIVE_ID = 9;
    public static final int BACK_RIGHT_DRIVE_ID = 7;

    public static final STSmaxConfig FRONT_LEFT_STEER = new STSmaxConfig();
    public static final STSmaxConfig FRONT_RIGHT_STEER = new STSmaxConfig();
    public static final STSmaxConfig BACK_LEFT_STEER = new STSmaxConfig();
    public static final STSmaxConfig BACK_RIGHT_STEER = new STSmaxConfig();

    public static void configureMotors() {

        FRONT_LEFT_STEER.name = "FRONT_LEFT_STEER";
        FRONT_RIGHT_STEER.name = "FRONT_RIGHT_STEER";
        BACK_LEFT_STEER.name = "BACK_LEFT_STEER";
        BACK_RIGHT_STEER.name = "BACK_RIGHT_STEER";

        FRONT_RIGHT_DRIVE.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        BACK_RIGHT_DRIVE.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        FRONT_LEFT_DRIVE.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        BACK_LEFT_DRIVE.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        FRONT_LEFT_STEER.id = 6;
        FRONT_RIGHT_STEER.id = 4;
        BACK_LEFT_STEER.id = 10;
        BACK_RIGHT_STEER.id = 8;
    }

    public static void configureDriveTalon(TalonFX motor) {
        Slot0Configs velConstants = new Slot0Configs();
        velConstants.kP = 0.75; // 0.4 FIXME
        velConstants.kS = 0.0; // 0.25 FIXME
        velConstants.kV = 12.0 / (MiscUtil.DTmetersToRot(MAX_PHYSICAL_SPEED_M_S));
        velConstants.kA = 20.0 / (MiscUtil.DTmetersToRot(MAX_ACCEL));
        motor.getConfigurator().apply(velConstants);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.SensorToMechanismRatio = (50.0 / 15.0) * (16.0 / 28.0) * (45.0 / 15.0);
        motor.getConfigurator().apply(feedbackConfigs);

        CurrentLimitsConfigs currConfigs = new CurrentLimitsConfigs();
        currConfigs.StatorCurrentLimitEnable = true;
        currConfigs.SupplyCurrentLimitEnable = true;
        currConfigs.StatorCurrentLimit = 65;
        currConfigs.SupplyCurrentLimit = 70;
        currConfigs.SupplyCurrentThreshold = 90;
        currConfigs.SupplyTimeThreshold = 0.25;
        motor.getConfigurator().apply(currConfigs);

        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    // public static STSmaxConfig configureDriveNeo(STSmaxConfig config) {

    //     config.kP = 0.02;
    //     config.kFF = 1 / DrivetrainConstants.MAX_PHYSICAL_SPEED_M_S;
    //     config.kD = 0.0;

    //     config.currentLimit = 40;

    //     config.isRotational = false;

    //     config.gearing = (16.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
    //     config.finalDiameterMeters = WHEEL_DIAMETER_METERS;
    //     config.idleMode = IdleMode.kBrake;

    //     return config;
    // }

    public static STSmaxConfig configureSteerNeo(STSmaxConfig config) {
        config.currentLimit = 20;
        config.gearing = (1.0 / (150.0 / 7.0));
        config.isRotational = true;
        config.kP = 7.0;
        config.kD = 0.15; // 1.6
        config.maxVel = (5676 * config.gearing * 2 * Math.PI) / (60.0);
        config.maxAccel = config.maxVel * 1000;
        config.inverted = true;
        config.idleMode = IdleMode.kBrake;
        return config;
    }

    public static void configureCanCoder(CANcoder encoder, double offset) {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = offset;

        encoder.getConfigurator().apply(config);
    }

}
