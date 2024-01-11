package frc.robot.util;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class SteelTalonsSparkMaxFlywheel {

    private CANSparkMax smax;
    private RelativeEncoder smaxEnc;
    private SparkPIDController smaxPID;
    private STSmaxConfig config;
    private double setPoint = 0;


    public SteelTalonsSparkMaxFlywheel(STSmaxConfig config) {
        config.isRotational = false;
        this.config = config;
        smax = new CANSparkMax(config.id, MotorType.kBrushless);
        Timer.delay(0.15);
        smax.setInverted(config.inverted);
        smax.setSmartCurrentLimit(config.currentLimit);
        smax.setIdleMode(config.idleMode);
        smaxEnc = smax.getEncoder();
        smaxEnc.setMeasurementPeriod(10);
        double positionConv = (config.gearing * config.finalDiameterMeters * Math.PI);
        //M - M/s
        smaxEnc.setPositionConversionFactor(positionConv);
        smaxEnc.setVelocityConversionFactor(positionConv / 60);
        smaxEnc.setPosition(0);
        smaxPID = smax.getPIDController();
        smaxPID.setP(config.kP);
        smaxPID.setD(config.kD);
        smaxPID.setFF(config.kFF);
        smaxPID.setI(config.kI);
        smax.burnFlash();
        Timer.delay(0.15);

    }

    public void setRaw(double percent) {
        smax.setVoltage(percent * smax.getBusVoltage());
    }

    public CANSparkMax getSmax() {
        return smax;
    }

    public void setSetpoint(double setPoint, double arbFF) {
        this.setPoint = setPoint;
        smaxPID.setReference(setPoint, ControlType.kVelocity, 0, arbFF);
    }

    public double getSetPoint() {
        return setPoint;
    }

    public void forceStop() {
        smax.setVoltage(0);
    }

    public double getPosition() {
        return smaxEnc.getPosition();
    }

    public void setPosition(double pos) {
        smaxEnc.setPosition(pos);
    }

    public double getVelocity() {
        return smaxEnc.getVelocity();
    }

    public double getError() {
        if (config.isRotational) {
            return new Rotation2d(setPoint).minus(new Rotation2d(getPosition())).getRadians();
        } else {
            return setPoint - getPosition();
        }
    }

    public void log() {
        String name = config.name;
        SteelTalonsLogger.post(name + ": Applied Output (%)", smax.getAppliedOutput());
        SteelTalonsLogger.post(name + ": Output Current (A)", smax.getOutputCurrent());
        SteelTalonsLogger.post(name + ": Temp (C)", smax.getMotorTemperature());
        SteelTalonsLogger.post(name + ": Is Braked? (Bool)", smax.getIdleMode().equals(IdleMode.kBrake));
        SteelTalonsLogger.post(name + ": Position (Meters)", getPosition());
        SteelTalonsLogger.post(name + ": Velocity (Meters/s)", getVelocity());
        SteelTalonsLogger.post(name + ": Setpoint (Meters)", getSetPoint());
        SteelTalonsLogger.post(name + ": Error (Meters)", getError());
    }
}
