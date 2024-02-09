package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.STSmaxConfig;

public class IntakeConstants {
    public static STSmaxConfig ROLLER_CONFIG = new STSmaxConfig();
    public static STSmaxConfig PIVOT_CONFIG = new STSmaxConfig();

    private static final int ROLLER_MOTOR_ID = 17;
    private static final int PIVOT_MOTOR_ID = 18;

    public static final Rotation2d STOWED_POS = new Rotation2d(Units.degreesToRadians(-26.0));
    public static final Rotation2d INTAKING_POS = new Rotation2d(Units.degreesToRadians(-209.5)); //-210
    public static final Rotation2d HARDSTOP_POS = new Rotation2d(Units.degreesToRadians(0.0));

    public static final double INTAKE_SPEED_INTAKING = 17.0;
    public static final double INTAKE_SPEED_STOPPED = 0.0;

    public static final double PIVOT_TOLERANCE_RAD = (Units.degreesToRadians(2));

    public static final double kS = 0.00; //FIXME
    public static final double kG = 0.58;
    public static final double kV = 0.55;
    public static final double kA = 0.01;

    public static final int BEAM_BREAKER_PORT = 0;

    public static void configureIntake() {
        ROLLER_CONFIG.name = "Intake Roller";
        PIVOT_CONFIG.name = "Intake Pivot";

        ROLLER_CONFIG.isRotational = false;
        PIVOT_CONFIG.isRotational = true;

        PIVOT_CONFIG.currentLimit = 30;

        ROLLER_CONFIG.id = ROLLER_MOTOR_ID;
        PIVOT_CONFIG.id = PIVOT_MOTOR_ID;

        ROLLER_CONFIG.gearing = 18.0/24.0;
        ROLLER_CONFIG.finalDiameterMeters = Units.inchesToMeters(2);
        PIVOT_CONFIG.gearing = (12.0/52.0) * (16.0/52.0) * (16.0/32.0);

        ROLLER_CONFIG.maxVel = (5676 * ROLLER_CONFIG.gearing * ROLLER_CONFIG.finalDiameterMeters * 2 * Math.PI) / (60.0);
        ROLLER_CONFIG.maxAccel = ROLLER_CONFIG.maxVel * 4;
        PIVOT_CONFIG.maxVel = (5676 * PIVOT_CONFIG.gearing * 2 * Math.PI) / (60.0);
        PIVOT_CONFIG.maxAccel = PIVOT_CONFIG.maxVel * 2.5;

        ROLLER_CONFIG.inverted = false;
        PIVOT_CONFIG.inverted = true;

        ROLLER_CONFIG.idleMode = IdleMode.kCoast;
        PIVOT_CONFIG.idleMode = IdleMode.kBrake;

        ROLLER_CONFIG.kP = 0.0;
        ROLLER_CONFIG.kFF = 1.0 / ROLLER_CONFIG.maxVel;
        PIVOT_CONFIG.kP = 9.0; // 5
        PIVOT_CONFIG.kD = 0.4;
    }
}