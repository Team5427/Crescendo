package frc.robot.subsystems.Shooter;

import frc.robot.util.STSmaxConfig;

public class ShooterConstants {
    
    public static STSmaxConfig shooterPivotConfig = new STSmaxConfig();
    public static STSmaxConfig shooterLeftFlywheelConfig = new STSmaxConfig();
    public static STSmaxConfig shooterRightFlywheelConfig = new STSmaxConfig();

    private static final int SHOOTER_PIVOT_MOTOR_ID = 0;
    private static final int SHOOTER_LEFT_FLYWHEEL_MOTOR_ID = 0;
    private static final int SHOOTER_RIGHT_FLYWHEEL_MOTOR_ID = 0;

    public static void configureShooter() {
        shooterPivotConfig.name = "Shooter Pivot";
        shooterLeftFlywheelConfig.name = "Shooter Left Flywheel";
        shooterRightFlywheelConfig.name = "Shooter Right Flywheel";

        shooterPivotConfig.id = SHOOTER_PIVOT_MOTOR_ID;
        shooterLeftFlywheelConfig.id = SHOOTER_LEFT_FLYWHEEL_MOTOR_ID;
        shooterRightFlywheelConfig.id = SHOOTER_RIGHT_FLYWHEEL_MOTOR_ID;

        shooterPivotConfig.inverted = false;
        shooterLeftFlywheelConfig.inverted = true;
        shooterRightFlywheelConfig.inverted = false;
    }

}
