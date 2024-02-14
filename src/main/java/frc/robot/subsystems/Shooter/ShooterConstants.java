package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;
import frc.robot.util.STSmaxConfig;

public class ShooterConstants {
    
    public static STSmaxConfig shooterPivotConfig = new STSmaxConfig();
    public static STSmaxConfig feederRollerConfig = new STSmaxConfig();
    public static STSmaxConfig shooterLeftFlywheelConfig = new STSmaxConfig();
    public static STSmaxConfig shooterRightFlywheelConfig = new STSmaxConfig();
    public static STSmaxConfig ampPivotConfig = new STSmaxConfig();

    private static final int SHOOTER_PIVOT_MASTER_MOTOR_ID = 19;
    public static final int SHOOTER_PIVOT_SLAVE_MOTOR_ID = 20;
    private static final int FEEDER_ROLLER_MOTOR_ID = 21;
    private static final int SHOOTER_LEFT_FLYWHEEL_MOTOR_ID = 22;
    private static final int SHOOTER_RIGHT_FLYWHEEL_MOTOR_ID = 23;
    private static final int AMP_PIVOT_MOTOR_ID = 24;

    private static final double FEEDER_ROLLER_DIAMETER_METERS = Units.inchesToMeters(1.0);
    private static final double SHOOTER_FLYWHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);

    public static void configureShooter() {
        shooterPivotConfig.name = "Shooter Pivot";
        feederRollerConfig.name = "Feeder Roller";
        shooterLeftFlywheelConfig.name = "Shooter Left Flywheel";
        shooterRightFlywheelConfig.name = "Shooter Right Flywheel";
        ampPivotConfig.name = "Amp Pivot";

        shooterPivotConfig.id = SHOOTER_PIVOT_MASTER_MOTOR_ID;
        feederRollerConfig.id = FEEDER_ROLLER_MOTOR_ID;
        shooterLeftFlywheelConfig.id = SHOOTER_LEFT_FLYWHEEL_MOTOR_ID;
        shooterRightFlywheelConfig.id = SHOOTER_RIGHT_FLYWHEEL_MOTOR_ID;
        ampPivotConfig.id = AMP_PIVOT_MOTOR_ID;

        shooterPivotConfig.currentLimit = 30;
        feederRollerConfig.currentLimit = 60;
        shooterLeftFlywheelConfig.currentLimit = 40;
        shooterRightFlywheelConfig.currentLimit = 40;
        ampPivotConfig.currentLimit = 30;

        shooterPivotConfig.inverted = false;
        feederRollerConfig.inverted = false;
        shooterLeftFlywheelConfig.inverted = true;
        shooterRightFlywheelConfig.inverted = false;
        ampPivotConfig.inverted = false;

        shooterPivotConfig.isRotational = true;
        feederRollerConfig.isRotational = false;
        shooterLeftFlywheelConfig.isRotational = false;
        shooterRightFlywheelConfig.isRotational = false;
        ampPivotConfig.isRotational = true;

        shooterPivotConfig.gearing = (1.0 / 9.0) * (30.0 / 64.0) * (12.0 / 58.0);
        feederRollerConfig.gearing = 1.0 / 9.0;
        shooterLeftFlywheelConfig.gearing = 1.0;
        shooterRightFlywheelConfig.gearing = 1.0;
        ampPivotConfig.gearing = 1.0 / 100.0; //FIXME

        feederRollerConfig.finalDiameterMeters = FEEDER_ROLLER_DIAMETER_METERS;
        shooterLeftFlywheelConfig.finalDiameterMeters = SHOOTER_FLYWHEEL_DIAMETER_METERS;
        shooterRightFlywheelConfig.finalDiameterMeters = SHOOTER_FLYWHEEL_DIAMETER_METERS;

        shooterPivotConfig.maxVel = shooterPivotConfig.getStandardMaxVelocity();
        shooterPivotConfig.maxAccel = shooterPivotConfig.maxVel * 2;

        feederRollerConfig.maxVel = feederRollerConfig.getStandardMaxVelocity() * (11000.0 / 5676.0);
        feederRollerConfig.maxAccel = feederRollerConfig.maxVel * 4.0;

        shooterLeftFlywheelConfig.maxVel = shooterLeftFlywheelConfig.getStandardMaxVelocity();
        shooterLeftFlywheelConfig.maxAccel = shooterLeftFlywheelConfig.maxVel * 4.0;

        shooterRightFlywheelConfig.maxVel = shooterRightFlywheelConfig.getStandardMaxVelocity();
        shooterRightFlywheelConfig.maxAccel = shooterRightFlywheelConfig.maxVel * 4.0;

        ampPivotConfig.maxVel = ampPivotConfig.getStandardMaxVelocity() * (11000.0 / 5676.0);
        ampPivotConfig.maxAccel = ampPivotConfig.maxVel * 4.0;

        shooterPivotConfig.idleMode = IdleMode.kBrake;
        feederRollerConfig.idleMode = IdleMode.kCoast;
        shooterLeftFlywheelConfig.idleMode = IdleMode.kCoast;
        shooterRightFlywheelConfig.idleMode = IdleMode.kCoast;
        ampPivotConfig.idleMode = IdleMode.kBrake;

        shooterPivotConfig.kP = 0.0;
        feederRollerConfig.kP = 0.0;
        feederRollerConfig.kFF = 1.0 / feederRollerConfig.getStandardMaxVelocity();
        shooterLeftFlywheelConfig.kP = 0.0;
        shooterLeftFlywheelConfig.kFF = 0.9 / shooterLeftFlywheelConfig.getStandardMaxVelocity();
        shooterRightFlywheelConfig.kP = 0.0;
        shooterRightFlywheelConfig.kFF = 0.9 / shooterRightFlywheelConfig.getStandardMaxVelocity();
        ampPivotConfig.kP = 0.0;
    }

}
