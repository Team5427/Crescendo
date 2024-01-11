package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.util.STSmaxConfig;

public class ArmConstants {
    
    public static STSmaxConfig ARM_MASTER = new STSmaxConfig();

    private static final int ARM_MOTOR_ID = 0;

    public static void configureArmMotor() {
        ARM_MASTER.name = "Arm Motor";
        ARM_MASTER.gearing = 0.0/0.0;
        ARM_MASTER.inverted = false;
        ARM_MASTER.id = ARM_MOTOR_ID;

        ARM_MASTER.kP = 0.0;
        ARM_MASTER.kD = 0.0;

        ARM_MASTER.idleMode = IdleMode.kBrake;
        ARM_MASTER.isRotational = true;
        
        ARM_MASTER.maxVel = (5676 * ARM_MASTER.gearing * 2 * Math.PI) / (60.0);
        ARM_MASTER.maxAccel = ARM_MASTER.maxVel * 4;
    }

}
