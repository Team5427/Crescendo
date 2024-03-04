package frc.robot.util.SmaxProfiles;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.MiscUtil;
import frc.robot.util.STSmaxConfig;
import frc.robot.util.SteelTalonsLogger;

public class SteelTalonsSparkMaxServo {

    private CANSparkMax smax;
    private RelativeEncoder smaxEnc;
    private ProfiledPIDController smaxController;
    private STSmaxConfig config;
    private double setPoint = 0;

    public SteelTalonsSparkMaxServo(STSmaxConfig config) {
        this.config = config;
        smax = new CANSparkMax(config.id, MotorType.kBrushless);
        Timer.delay(0.15);
        smax.setInverted(config.inverted);
        smax.setSmartCurrentLimit(config.currentLimit);
        smax.setIdleMode(config.idleMode);
        smaxEnc = smax.getEncoder();
        smaxEnc.setMeasurementPeriod(10);
        double positionConv = config.isRotational ? (2 * Math.PI * config.gearing) : (config.gearing * config.finalDiameterMeters * Math.PI);
        //Rotational subsystem: Rad - Rad/s --- Linear subsystem: M - M/s
        smaxEnc.setPositionConversionFactor(positionConv);
        smaxEnc.setVelocityConversionFactor(positionConv / 60.0);
        smaxEnc.setPosition(0);
        smaxController = new ProfiledPIDController(config.kP, config.kI, config.kD, 
            new Constraints(config.maxVel, config.maxAccel)
        );
        if (config.isRotational) {
            smaxController.enableContinuousInput(-Math.PI, Math.PI);
        }
        MiscUtil.doPeriodicFrame(smax);
        smax.burnFlash();
        Timer.delay(0.15);
    }

    public SteelTalonsSparkMaxServo(STSmaxConfig config, int id) {
        this.config = config;
        smax = new CANSparkMax(id, MotorType.kBrushless);
        Timer.delay(0.15);
        smax.setInverted(config.inverted);
        smax.setSmartCurrentLimit(config.currentLimit);
        smax.setIdleMode(config.idleMode);
        smaxEnc = smax.getEncoder();
        smaxEnc.setMeasurementPeriod(10);
        double positionConv = config.isRotational ? (2 * Math.PI * config.gearing) : (config.gearing * config.finalDiameterMeters * Math.PI);
        //Rotational subsystem: Rad - Rad/s --- Linear subsystem: M - M/s
        smaxEnc.setPositionConversionFactor(positionConv);
        smaxEnc.setVelocityConversionFactor(positionConv / 60.0);
        smaxEnc.setPosition(0);
        smaxController = new ProfiledPIDController(config.kP, config.kI, config.kD, 
            new Constraints(config.maxVel, config.maxAccel)
        );
        if (config.isRotational) {
            smaxController.enableContinuousInput(-Math.PI, Math.PI);
        }
        smax.burnFlash();
        Timer.delay(0.15);
    }

    public void setRaw(double percent) {
        smaxController.reset(getPosition());
        smax.set(percent);
    }

    public void setAccel(double accel) {
        smaxController.setConstraints(new Constraints(config.maxVel, accel));
    }

    public CANSparkMax getSmax() {
        return smax;
    }

    public void setSetpoint(double setPoint, double arbFF) {
        this.setPoint = setPoint;
        smaxController.reset(getPosition());
        smax.setVoltage(smaxController.calculate(getPosition(), setPoint) + arbFF + config.kFF * setPoint);
    }

    public double getSetPoint() {
        return setPoint;
    }

    public void forceStop() {
        smax.setVoltage(0);
    }

    public void disableContinuousInput() {
        smaxController.disableContinuousInput();
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

    public double getSetpointVelocity() {
        return smaxController.getSetpoint().velocity;
    }

    public double getError() {
        if (config.isRotational) {
            return new Rotation2d(setPoint).minus(new Rotation2d(getPosition())).getRadians();
        } else {
            return setPoint - getPosition();
        }
    }

    public void resetController() {
        smaxController.reset(getPosition());
    }

    public void log() {
        String name = config.name;
        SteelTalonsLogger.post(name + ": Applied Output (%)", smax.getAppliedOutput());
        SteelTalonsLogger.post(name + ": Output Current (A)", smax.getOutputCurrent());
        SteelTalonsLogger.post(name + ": Temp (C)", smax.getMotorTemperature());
        SteelTalonsLogger.post(name + ": Is Braked? (Bool)", smax.getIdleMode().equals(IdleMode.kBrake));
        SteelTalonsLogger.post(name + ": Position (rad or Meters)", getPosition());
        SteelTalonsLogger.post(name + ": Velocity (rad/s or Meters/s)", getVelocity());
        SteelTalonsLogger.post(name + ": Setpoint (rad or Meters)", getSetPoint());
        SteelTalonsLogger.post(name + ": Error (rad or Meters)", getError());
    }
}
