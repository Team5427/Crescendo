package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.util.STSmaxConfig;

public class ElevatorConstants {
    
    public static STSmaxConfig ELEVATOR_MASTER = new STSmaxConfig();

    private static final int ELEVATOR_MOTOR_ID = 0;

    public static void configureElevatorMotor() {
        ELEVATOR_MASTER.name = "Elevator Motor";
        ELEVATOR_MASTER.gearing = 0.0/0.0f;
        ELEVATOR_MASTER.inverted = false;
        ELEVATOR_MASTER.id = ELEVATOR_MOTOR_ID;

        ELEVATOR_MASTER.kP = 0.0;
        ELEVATOR_MASTER.kD = 0.0;

        ELEVATOR_MASTER.isRotational = false;
        ELEVATOR_MASTER.idleMode = IdleMode.kBrake;

        ELEVATOR_MASTER.maxVel = (5676 * ELEVATOR_MASTER.gearing * 2 * Math.PI) / (60.0);
        ELEVATOR_MASTER.maxAccel = ELEVATOR_MASTER.maxVel * 4;
    }

}
