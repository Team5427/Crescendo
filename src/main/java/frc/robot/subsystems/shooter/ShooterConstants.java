package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.lib.drivers.CANDeviceId;
import frc.robot.lib.drivers.ComplexGearRatio;
import frc.robot.lib.motors.MotorConfiguration;
import frc.robot.lib.motors.MotorConfiguration.IdleState;
import frc.robot.lib.motors.MotorConfiguration.MotorMode;

public class ShooterConstants {
    public static final Rotation2d kStow = new Rotation2d(30); // UPDATE
    public static final Rotation2d kHandoff = new Rotation2d(10); // UPDATE
    public static final Rotation2d kDefaultShooting = new Rotation2d(80); // UPDATE
    public static final Rotation2d kShootAmp = new Rotation2d(90); // UPDATE

    public static final double kFlywheelTolerance = .01; // UPDATE

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

    public static void createConfigs() {
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
