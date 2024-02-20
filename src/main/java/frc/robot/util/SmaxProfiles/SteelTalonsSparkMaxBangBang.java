package frc.robot.util.SmaxProfiles;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.MiscUtil;
import frc.robot.util.STSmaxConfig;
import frc.robot.util.SteelTalonsLogger;

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
        smax.setIdleMode(IdleMode.kCoast);
        smaxEnc = smax.getEncoder();
        smaxEnc.setMeasurementPeriod(10);
        //M - M/s
        smaxEnc.setPositionConversionFactor(1.0);
        smaxEnc.setVelocityConversionFactor(1.0);
        smaxEnc.setPosition(0);

        controller = new BangBangController(100); //tolerance in RPM
        MiscUtil.doPeriodicFrame(smax);

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
        // smax.setVoltage(controller.calculate(getVelocity(), this.setPoint) * 12 + config.kFF * setPoint * 12);
        smax.setVoltage(controller.calculate(getVelocity(), setPoint) + config.kFF * setPoint * 12);    
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
        return Math.abs(setPoint - getVelocity());
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
