package frc.robot.util;

import com.revrobotics.CANSparkBase.IdleMode;

public class STSmaxConfig {
    public String name;
    public int id;
    public boolean inverted;
    public double gearing;
    public boolean isRotational;
    public double finalDiameterMeters; //only useful for linear motors
    public int currentLimit;
    public double kP, kD, kI, kFF;
    public double maxAccel, maxVel;
    public IdleMode idleMode;
    public STSmaxConfig() {
        name = "NOT CONSTRUCUTED";
        id = 0;
        inverted = false;
        gearing = 1.0;
        isRotational = true;
        finalDiameterMeters = 1.0 / Math.PI; //Circumference is 1
        currentLimit = 40;
        maxAccel = 0.0;
        maxVel = 0.0;
        idleMode = IdleMode.kBrake;
    }
}
