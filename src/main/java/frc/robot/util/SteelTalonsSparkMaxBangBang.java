package frc.robot.util;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class SteelTalonsSparkMaxBangBang {

    private CANSparkMax smax;
    private RelativeEncoder smaxEnc;
    // private SparkPIDController smaxPID;
    private BangBangController controller;
    private STSmaxConfig config;
    private double setPoint = 0;


    public SteelTalonsSparkMaxBangBang(STSmaxConfig config) {
        config.isRotational = false;
        this.config = config;
        smax = new CANSparkMax(config.id, MotorType.kBrushless);
        Timer.delay(0.15);
        smax.setInverted(config.inverted);
        smax.setSmartCurrentLimit(config.currentLimit);
        smax.setIdleMode(config.idleMode);
        smaxEnc = smax.getEncoder();
        smaxEnc.setMeasurementPeriod(10);
        //M - M/s
        smaxEnc.setPositionConversionFactor(1.0);
        smaxEnc.setVelocityConversionFactor(1.0);
        smaxEnc.setPosition(0);
        smax.burnFlash();

        controller = new BangBangController(100); //tolerance in RPM
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
        smax.setVoltage(controller.calculate(getVelocity(), this.setPoint));
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
