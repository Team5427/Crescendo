package frc.robot.util.SmaxProfiles;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.MiscUtil;
import frc.robot.util.STSmaxConfig;
import frc.robot.util.SteelTalonsLogger;

public class SteelTalonsSparkMaxFlywheel {

    private CANSparkMax smax;
    private RelativeEncoder smaxEnc;
    private SparkPIDController smaxPID;
    private STSmaxConfig config;
    private double setPoint = 0;
    private SlewRateLimiter accelLimiter;
    private boolean disableLimiter = false;


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
        MiscUtil.doPeriodicFrame(smax);
        smax.burnFlash();
        accelLimiter = new SlewRateLimiter(config.maxAccel);
        Timer.delay(0.15);

    }

    public void setRaw(double percent) {
        resetLimiter();
        smax.setVoltage(percent * smax.getBusVoltage());
    }

    public CANSparkMax getSmax() {
        return smax;
    }

    public void setSetpoint(double setPoint, double arbFF) {
        this.setPoint = !disableLimiter ? accelLimiter.calculate(setPoint) : setPoint;
        smaxPID.setReference(this.setPoint, ControlType.kVelocity, 0, arbFF);
    }

    public double getSetPoint() {
        return setPoint;
    }

    public void forceStop() {
        resetLimiter();
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
            return setPoint - getVelocity();
        }
    }

    public void resetLimiter() {
        accelLimiter.reset(getVelocity());
    }

    public void disableLimiter() {
        this.disableLimiter = true;
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
