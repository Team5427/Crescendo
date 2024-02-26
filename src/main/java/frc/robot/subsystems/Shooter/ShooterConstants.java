package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.util.MiscUtil;
import frc.robot.util.STSmaxConfig;

public class ShooterConstants {
    
    public static STSmaxConfig shooterPivotConfig = new STSmaxConfig();
    public static STSmaxConfig feederRollerConfig = new STSmaxConfig();
    public static STSmaxConfig shooterLeftFlywheelConfig = new STSmaxConfig();
    public static STSmaxConfig shooterRightFlywheelConfig = new STSmaxConfig();
    public static STSmaxConfig ampPivotConfig = new STSmaxConfig();

    private static final int SHOOTER_PIVOT_MASTER_MOTOR_ID = 24;
    public static final int SHOOTER_PIVOT_SLAVE_MOTOR_ID = 23;
    private static final int FEEDER_ROLLER_MOTOR_ID = 22;
    private static final int SHOOTER_LEFT_FLYWHEEL_MOTOR_ID = 19;
    private static final int SHOOTER_RIGHT_FLYWHEEL_MOTOR_ID = 20;
    private static final int AMP_PIVOT_MOTOR_ID = 21;
    
    public static final int BEAM_BREAKER_PORT = 0;
    public static final int SIDE_BEAM_BREAKER_PORT = 2;

    private static final double FEEDER_ROLLER_DIAMETER_METERS = Units.inchesToMeters(1.25);

    public static final double FLYWHEEL_TOLERANCE_RPM = 100.0;
    public static final double FEEDER_TOLERANCE_M_S = 0.2;
    public static final Rotation2d PIVOT_TOLERANCE_RAD = new Rotation2d(Math.toRadians(0.25));
    public static final Rotation2d AMP_TOLERANCE_RAD = new Rotation2d(Math.toRadians(1.25));

    public static final Rotation2d SHOOTER_PIVOT_HARDSTOP = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d SHOOTER_PIVOT_STOW = Rotation2d.fromDegrees(-5.0);
    public static final Rotation2d SHOOTER_PIVOT_HANDOFF = Rotation2d.fromDegrees(-45.0);
    public static final Rotation2d SHOOTER_PIVOT_AMP = Rotation2d.fromDegrees(-3.0);
    public static final Rotation2d SHOOTER_PIVOT_ACTIVE = Rotation2d.fromDegrees(-20.0);
    public static final Rotation2d SHOOTER_PIVOT_MAX_ROT = Rotation2d.fromDegrees(-55.0);

    public static final Rotation2d AMP_HARDSTOP = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d AMP_DEPLOYED = Rotation2d.fromDegrees(-160.0);

    public static final double FLYWHEEL_STATIC_SPEED_RPM = 1000;
    public static final double FLYWHEEL_AMP_SPEED_RPM = 2000;

    public static final double FEEDER_HOLD_SPEED = 0.0;
    public static final double FEEDER_BUMP_SPEED = 0.1;
    public static final double FEEDER_FEED_SPEED = 2.25;
    public static final double FEEDER_INTAKE_SPEED = 1.0;

    public static InterpolatingTreeMap<Double, ShootingConfiguration> SHOOTER_PIVOT_TARGET_MAP = new InterpolatingTreeMap<Double, ShootingConfiguration>(
        MiscUtil.getInversePoseInterpolator(), 
        MiscUtil.getShooterInterpolator()
    );

    public static void targetMap() {


        SHOOTER_PIVOT_TARGET_MAP.put(
            0.0, //DISTANCE METERS
            new ShootingConfiguration(
                Rotation2d.fromDegrees(-5.0), //SHOOTER ROTATION 
                4300, //LEFT RPM
                5300 //RIGHT RPM
            )
        );

        SHOOTER_PIVOT_TARGET_MAP.put(
            1.0, //DISTANCE METERS
            new ShootingConfiguration(
                Rotation2d.fromDegrees(-10.0), //SHOOTER ROTATION 
                4300, //LEFT RPM
                5300 //RIGHT RPM
            )
        );

        SHOOTER_PIVOT_TARGET_MAP.put(
            2.0, //DISTANCE METERS
            new ShootingConfiguration(
                Rotation2d.fromDegrees(-18.0), //SHOOTER ROTATION 
                4600, //LEFT RPM
                5400 //RIGHT RPM
            )
        );

        SHOOTER_PIVOT_TARGET_MAP.put(
            3.0, //DISTANCE METERS
            new ShootingConfiguration(
                Rotation2d.fromDegrees(-20.0), //SHOOTER ROTATION 
                4300, //LEFT RPM
                5500 //RIGHT RPM
            )
        );

        SHOOTER_PIVOT_TARGET_MAP.put(
            4.0, //DISTANCE METERS
            new ShootingConfiguration(
                Rotation2d.fromDegrees(-22.0), //SHOOTER ROTATION 
                5000, //LEFT RPM
                5500 //RIGHT RPM
            )
        );

    }

    public static InterpolatingDoubleTreeMap SHOOTER_OTF_OFFSET_MAP = new InterpolatingDoubleTreeMap();

    public static void targetOffsetMap() { //INPUT: m/s, OUTPUT: degrees offset
        SHOOTER_OTF_OFFSET_MAP.put(
            0.0,
            0.0 
        );

        SHOOTER_OTF_OFFSET_MAP.put(
            1.0, 
            -3.0
        );

        SHOOTER_OTF_OFFSET_MAP.put(
            2.0, 
            -8.0
        );

        SHOOTER_OTF_OFFSET_MAP.put(
            3.0, 
            -11.0
        );
    }

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
        feederRollerConfig.currentLimit = 30;
        shooterLeftFlywheelConfig.currentLimit = 50;
        shooterRightFlywheelConfig.currentLimit = 50;
        ampPivotConfig.currentLimit = 20;

        shooterPivotConfig.inverted = false;
        feederRollerConfig.inverted = false;
        shooterLeftFlywheelConfig.inverted = true;
        shooterRightFlywheelConfig.inverted = false;
        ampPivotConfig.inverted = true;

        shooterPivotConfig.isRotational = true;
        feederRollerConfig.isRotational = false;
        shooterLeftFlywheelConfig.isRotational = false;
        shooterRightFlywheelConfig.isRotational = false;
        ampPivotConfig.isRotational = true;

        shooterPivotConfig.gearing = (1.0 / 9.0) * (30.0 / 64.0) * (12.0 / 58.0);
        feederRollerConfig.gearing = 1.0 / 7.0;
        shooterLeftFlywheelConfig.gearing = 1.0;
        shooterRightFlywheelConfig.gearing = 1.0;
        ampPivotConfig.gearing = 1.0 / 100.0;

        feederRollerConfig.finalDiameterMeters = FEEDER_ROLLER_DIAMETER_METERS;

        shooterPivotConfig.maxVel = shooterPivotConfig.getStandardMaxVelocity();
        shooterPivotConfig.maxAccel = shooterPivotConfig.maxVel * 3;

        feederRollerConfig.maxVel = feederRollerConfig.getStandardMaxVelocity() * (11000.0 / 5676.0);
        feederRollerConfig.maxAccel = feederRollerConfig.maxVel * 4.0;

        shooterLeftFlywheelConfig.maxVel = shooterLeftFlywheelConfig.getStandardMaxVelocity();
        shooterLeftFlywheelConfig.maxAccel = shooterLeftFlywheelConfig.maxVel * 2.0;

        shooterRightFlywheelConfig.maxVel = shooterRightFlywheelConfig.getStandardMaxVelocity();
        shooterRightFlywheelConfig.maxAccel = shooterRightFlywheelConfig.maxVel * 2.0;

        ampPivotConfig.maxVel = ampPivotConfig.getStandardMaxVelocity() * (11000.0 / 5676.0);
        ampPivotConfig.maxAccel = ampPivotConfig.maxVel * 4;

        shooterPivotConfig.idleMode = IdleMode.kCoast; // Use Brake
        feederRollerConfig.idleMode = IdleMode.kBrake;
        shooterLeftFlywheelConfig.idleMode = IdleMode.kCoast;
        shooterRightFlywheelConfig.idleMode = IdleMode.kCoast;
        ampPivotConfig.idleMode = IdleMode.kCoast;

        shooterPivotConfig.kP = 40.0;
        feederRollerConfig.kP = 0.0;
        feederRollerConfig.kFF = 1.0 / feederRollerConfig.getStandardMaxVelocity();
        shooterLeftFlywheelConfig.kP = 0.0;
        shooterLeftFlywheelConfig.kFF = 1.0 / 5676.0;
        shooterRightFlywheelConfig.kP = 0.0;
        shooterRightFlywheelConfig.kFF = 1.0 / 5676.0;
        ampPivotConfig.kP = 10.0;

        targetMap();
        targetOffsetMap();

    }

}
