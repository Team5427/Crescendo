package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;
import frc.robot.util.STSmaxConfig;

/*
 * Preliminary Code
 * Review over before implementing
 * Need to add pivot for shooter; Will add when localization and shooters are made
*/

public class ShooterConstants {
    public static STSmaxConfig LEFT_MOTOR_CONFIG = new STSmaxConfig();
    public static STSmaxConfig RIGHT_MOTOR_CONFIG = new STSmaxConfig();

    private static final int RIGHT_MOTOR_ID = 10; //CHANGE
    private static final int LEFT_MOTOR_ID = 11; //CHANGE

    public static final int LEFT_MOTOR_BASE_RPM = 500;
    public static final int RIGHT_MOTOR_BASE_RPM = 500;

    public static final double RIGHT_MAX_MOTOR_RPM = 5676;
    public static final double LEFT_MAX_MOTOR_RPM = 5676;

    // public static final double SPEED_TOLERANCE 

    public static void configureShooter(){
        RIGHT_MOTOR_CONFIG.name = "Right Shooter";
        LEFT_MOTOR_CONFIG.name = "Left Shooter";

        LEFT_MOTOR_CONFIG.isRotational = false;
        RIGHT_MOTOR_CONFIG.isRotational = false;

        RIGHT_MOTOR_CONFIG.id = RIGHT_MOTOR_ID;
        LEFT_MOTOR_CONFIG.id = LEFT_MOTOR_ID;

        //Gearing?

        RIGHT_MOTOR_CONFIG.inverted = false;
        LEFT_MOTOR_CONFIG.inverted = true;

        RIGHT_MOTOR_CONFIG.idleMode = IdleMode.kCoast;
        LEFT_MOTOR_CONFIG.idleMode = IdleMode.kCoast;

        //k values?
    }
}
