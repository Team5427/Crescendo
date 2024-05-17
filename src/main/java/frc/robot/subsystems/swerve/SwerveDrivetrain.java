package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.lib.motors.MotorConfiguration;

public class SwerveDrivetrain {
    public static final MotorConfiguration[] driveMotorConfigurations = new MotorConfiguration[SwerveDriveTrainConstants.kDriveMotorIds.length];

    public static final MotorConfiguration[] steerMotorConfigurations = new MotorConfiguration[SwerveDriveTrainConstants.kSteerMotorIds.length];

    public static void configureDriveMotors(MotorConfiguration[] motorConfigs) {
        for (MotorConfiguration config : driveMotorConfigurations) {
            config.currentLimit = 70;
            // TODO
        }
    }

    public static void configureSteerMotors(MotorConfiguration[] motorConfigs) {

    }

    public static void configureCanCoders(CANcoder[] encoders, double offset) {
        // Iterate through all of the CanCoders
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = offset;

        for (CANcoder encoder : encoders) {
            encoder.getConfigurator().apply(config);
        }
    }
}
