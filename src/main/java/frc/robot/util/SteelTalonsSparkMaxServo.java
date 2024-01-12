package frc.robot.util;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;

public class SteelTalonsSparkMaxServo {

    private CANSparkMax smax;
    private RelativeEncoder smaxEnc;
    private SparkPIDController smaxPID;
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
        smax.burnFlash();
        Timer.delay(0.15);
    }

    public void setRaw(double percent) {
        smax.set(percent);
    }

    public CANSparkMax getSmax() {
        return smax;
    }

    public void setSetpoint(double setPoint, double arbFF) {
        this.setPoint = setPoint;
        smax.setVoltage(smaxController.calculate(getPosition(), setPoint) + arbFF + config.kFF * setPoint);
    }

    public double getSetPoint() {
        return setPoint;
    }

    public void forceStop() {
        smaxController.reset(getPosition());
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
