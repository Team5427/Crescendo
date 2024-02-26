package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class ClimberConstants {

    public static final int RIGHT_CLIMBER_ID = 28;
    public static final int LEFT_CLIMBER_ID = 27;

    private static final double axleDiameterMeters = Units.inchesToMeters(1);

    public static final double CLIMB_POSITION = Units.inchesToMeters(30.0);
    public static final double STOW_POSITION = Units.inchesToMeters(2.0);
    public static final double HARDSTOP_POSITION = Units.inchesToMeters(0.0);

    public static final double CLIMB_TOLERANCE = Units.inchesToMeters(1.0);

    public static void configureClimber(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40.0;

        Slot0Configs pidConfigs = new Slot0Configs();
        pidConfigs.kP = 0.0;
        pidConfigs.kI = 0.0;
        pidConfigs.kD = 0.0;

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.SensorToMechanismRatio = 25.0;

        motor.getConfigurator().apply(config);
        motor.getConfigurator().apply(pidConfigs);
        motor.getConfigurator().apply(feedbackConfigs);

        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setInverted(false);
    }

    public static double getClimberRotationsToMeters(double rotations) {
        return rotations * Math.PI * axleDiameterMeters;
    }
    
}
