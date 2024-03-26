package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.STSmaxConfig;

public class IntakeConstants {
    public static STSmaxConfig ROLLER_CONFIG = new STSmaxConfig();
    public static STSmaxConfig PIVOT_CONFIG = new STSmaxConfig();

    public static final int ROLLER_MOTOR_ID = 17;
    public static final int PIVOT_MOTOR_ID = 18;

    public static final double MAX_KRAKEN_ROLLER_SPEED_M_S = (6000.0 * Math.PI * Units.inchesToMeters(2.0) * (12.0/30.0)) / 60.0;

    public static final Rotation2d STOWED_POS = new Rotation2d(Units.degreesToRadians(-76.0));
    public static final Rotation2d INTAKING_POS = new Rotation2d(Units.degreesToRadians(-213.0)); //-210
    public static final Rotation2d HARDSTOP_POS = new Rotation2d(Units.degreesToRadians(0.0));
    public static final Rotation2d HANDOFF_POS = new Rotation2d(Units.degreesToRadians(-7.0));
    public static final Rotation2d BACKSHOT_POSE = new Rotation2d(Units.degreesToRadians(-40.0));

    public static final double INTAKE_SPEED_INTAKING = 6.0;
    public static final double INTAKE_SPEED_HOLD = 0.75;
    public static final double INTAKE_SPEED_EJECTING = -2.2;
    public static final double INTAKE_SPEED_EJECTING_MAX = -5.5;

    public static final double PIVOT_TOLERANCE_RAD = (Units.degreesToRadians(3.5));

    public static final int BEAM_BREAKER_PORT = 1;

    public static void configureIntake() {
        ROLLER_CONFIG.name = "Intake Roller";
        PIVOT_CONFIG.name = "Intake Pivot";

        ROLLER_CONFIG.isRotational = false;
        PIVOT_CONFIG.isRotational = true;

        PIVOT_CONFIG.currentLimit = 55;
        // ROLLER_CONFIG.currentLimit = 30;

        ROLLER_CONFIG.id = ROLLER_MOTOR_ID;
        PIVOT_CONFIG.id = PIVOT_MOTOR_ID;

        ROLLER_CONFIG.gearing = 12.0/30.0;
        ROLLER_CONFIG.finalDiameterMeters = Units.inchesToMeters(2);
        PIVOT_CONFIG.gearing = (12.0/52.0) * (16.0/52.0) * (16.0/32.0);

        ROLLER_CONFIG.maxVel = ROLLER_CONFIG.getStandardMaxVelocity();
        ROLLER_CONFIG.maxAccel = ROLLER_CONFIG.maxVel * 4;
        PIVOT_CONFIG.maxVel = PIVOT_CONFIG.getStandardMaxVelocity();
        PIVOT_CONFIG.maxAccel = PIVOT_CONFIG.maxVel * 2.0;

        ROLLER_CONFIG.inverted = false;
        PIVOT_CONFIG.inverted = true;

        ROLLER_CONFIG.idleMode = IdleMode.kCoast;
        PIVOT_CONFIG.idleMode = IdleMode.kBrake;

        ROLLER_CONFIG.kP = 0.0;
        ROLLER_CONFIG.kFF = 1.0 / ROLLER_CONFIG.maxVel;
        PIVOT_CONFIG.kP = 6.0; // 5
        PIVOT_CONFIG.kD = 0.3;
    }
}