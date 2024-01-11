package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.STSmaxConfig;

public class DrivetrainConstants {
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.88); //3.88
    public static final double TRACKWIDTH = Units.inchesToMeters(19.5);
    public static final double WHEELBASE = Units.inchesToMeters(19.5);

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEELBASE / 2, TRACKWIDTH / 2),
            new Translation2d(WHEELBASE / 2, -TRACKWIDTH / 2),
            new Translation2d(-WHEELBASE / 2, TRACKWIDTH / 2),
            new Translation2d(-WHEELBASE / 2, -TRACKWIDTH / 2));

    public static final int PIGEON_CAN_ID = 16;

    public static final double MAX_TRANSLATION_SPEED_M_S_TELEOP = Units.feetToMeters(16.2);
    public static final double MAX_ROTATION_SPEED_RAD_S_TELEOP = 8 * Math.PI;

    public static final double MAX_PHYSICAL_SPEED_M_S = (5676 * (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0) * WHEEL_DIAMETER_METERS * Math.PI) / (60.0);
    public static final double MAX_ACCEL = MAX_PHYSICAL_SPEED_M_S * 5;
    public static final double THRESHOLD_STOPPING_M_S = 0.3;

    public static final int FRONT_LEFT_CANCODER_ID = 12;
    public static final int FRONT_RIGHT_CANCODER_ID = 13;
    public static final int BACK_LEFT_CANCODER_ID = 14;
    public static final int BACK_RIGHT_CANCODER_ID = 15;

    public static final double FRONT_LEFT_OFFSET = 0.0;
    public static final double FRONT_RIGHT_OFFSET = 0.0;
    public static final double BACK_LEFT_OFFSET = 0.0;
    public static final double BACK_RIGHT_OFFSET = 0.0;

    public static final STSmaxConfig FRONT_LEFT_DRIVE = new STSmaxConfig();
    public static final STSmaxConfig FRONT_RIGHT_DRIVE = new STSmaxConfig();
    public static final STSmaxConfig BACK_LEFT_DRIVE = new STSmaxConfig();
    public static final STSmaxConfig BACK_RIGHT_DRIVE = new STSmaxConfig();

    public static final STSmaxConfig FRONT_LEFT_STEER = new STSmaxConfig();
    public static final STSmaxConfig FRONT_RIGHT_STEER = new STSmaxConfig();
    public static final STSmaxConfig BACK_LEFT_STEER = new STSmaxConfig();
    public static final STSmaxConfig BACK_RIGHT_STEER = new STSmaxConfig();

    public static void configureMotors() {
        FRONT_LEFT_DRIVE.name = "FRONT_LEFT_DRIVE";
        FRONT_RIGHT_DRIVE.name = "FRONT_RIGHT_DRIVE";
        BACK_LEFT_DRIVE.name = "BACK_LEFT_DRIVE";
        BACK_RIGHT_DRIVE.name = "BACK_RIGHT_DRIVE";

        FRONT_LEFT_STEER.name = "FRONT_LEFT_STEER";
        FRONT_RIGHT_STEER.name = "FRONT_RIGHT_STEER";
        BACK_LEFT_STEER.name = "BACK_LEFT_STEER";
        BACK_RIGHT_STEER.name = "BACK_RIGHT_STEER";

        FRONT_LEFT_DRIVE.id = 5;
        FRONT_RIGHT_DRIVE.id = 3;
        BACK_LEFT_DRIVE.id = 9;
        BACK_RIGHT_DRIVE.id = 7;

        FRONT_LEFT_STEER.id = 6;
        FRONT_RIGHT_STEER.id = 4;
        BACK_LEFT_STEER.id = 10;
        BACK_RIGHT_STEER.id = 8;
    }

    // public static void configureDriveTalon(TalonFX motor) {
    //     Slot0Configs velConstants = new Slot0Configs();
    //     velConstants.kP = 0.0; //FIXME
    //     velConstants.kS = 0.5; //FIXME
    //     velConstants.kV = 12 / (MAX_PHYSICAL_SPEED_M_S);
    //     motor.getConfigurator().apply(velConstants);

    //     FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    //     feedbackConfigs.SensorToMechanismRatio = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0) * WHEEL_DIAMETER_METERS * Math.PI;
    //     motor.getConfigurator().apply(feedbackConfigs);

    //     ClosedLoopRampsConfigs rampConfigs = new ClosedLoopRampsConfigs();
    //     rampConfigs.VoltageClosedLoopRampPeriod = 0.2; //secs it takes to ramp to max
    //     motor.getConfigurator().apply(rampConfigs);

    //     CurrentLimitsConfigs currConfigs = new CurrentLimitsConfigs();
    //     currConfigs.StatorCurrentLimitEnable = true;
    //     currConfigs.SupplyCurrentLimitEnable = true;
    //     currConfigs.StatorCurrentLimit = 60;
    //     currConfigs.SupplyCurrentLimit = 60;
    //     currConfigs.SupplyCurrentThreshold = 80;
    //     currConfigs.SupplyTimeThreshold = 1.5;
    //     motor.getConfigurator().apply(currConfigs);

    //     motor.setNeutralMode(NeutralModeValue.Brake);
    // }

    public static STSmaxConfig configureDriveNeo(STSmaxConfig config) {
        
        config.kP = 0.05;
        config.kD = 0.0;

        config.currentLimit = 80;

        config.isRotational = false;

        config.gearing = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
        config.finalDiameterMeters = WHEEL_DIAMETER_METERS;
        // config.maxVel = (5676 * config.gearing * 2 * Math.PI) / (60.0);
        // config.maxAccel = config.maxVel * 4;

        config.idleMode = IdleMode.kBrake;
        return config;
    }

    public static STSmaxConfig configureSteerNeo(STSmaxConfig config) {
        config.currentLimit = 20;
        config.gearing = (1.0 / (150.0/7.0));
        config.isRotational = true;
        config.kP = 2;
        config.kD = 0.0;
        // config.maxVel = (5676 * config.gearing * 2 * Math.PI) / (60.0);
        config.maxVel = 4800;
        config.maxAccel = config.maxVel * 4;
        config.inverted = true;
        config.idleMode = IdleMode.kCoast;
        return config;
    }

    public static void configureCanCoder(CANcoder encoder, double offset) {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.MagnetOffset = offset;

        encoder.getConfigurator().apply(config);
    }
    
}
