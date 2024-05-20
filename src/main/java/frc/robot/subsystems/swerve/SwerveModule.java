package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.lib.drivers.CANDeviceId;
import frc.robot.lib.kinematics.ConstrainedChassisSpeeds;
import frc.robot.lib.motors.MotorConfiguration;
import frc.robot.lib.motors.ProfiledSparkMax;
import frc.robot.lib.motors.SimpleSparkMax;
import frc.robot.lib.motors.SteelTalonFX;
import frc.robot.util.MathUtils;

public class SwerveModule {
    private SteelTalonFX driveMotor;
    private SimpleSparkMax steerMotor;

    private CANcoder encoder;
    private double deadzone = 0.0;

    public SwerveModule(MotorConfiguration driveMotorConfig, CANDeviceId driveMotorID,
            MotorConfiguration steerMotorConfig, CANDeviceId steerMotorID, CANcoder encoder) {
        this.encoder = encoder;
        this.driveMotor = new SteelTalonFX(driveMotorID);
        this.steerMotor = new SimpleSparkMax(steerMotorID);

        this.driveMotor.apply(driveMotorConfig);
        this.steerMotor.apply(steerMotorConfig);

        this.driveMotor.setEncoderPosition(0);
        this.steerMotor.setEncoderPosition(encoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                MathUtils.rotationsToMeters(driveMotor.getEncoderVelocity(),
                        DriveTrainConstants.kWheelDiamterMeters),
                new Rotation2d(steerMotor.getEncoderPosition()));
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
                MathUtils.rotationsToMeters(driveMotor.getEncoderVelocity(),
                        DriveTrainConstants.kWheelDiamterMeters),
                new Rotation2d(steerMotor.getEncoderPosition()));
    }

    public void setModuleState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state,
                Rotation2d.fromRotations(encoder.getPosition().getValueAsDouble()));

        double velocitySetpoint = MathUtils.metersToRotations(optimizedState.speedMetersPerSecond,
                DriveTrainConstants.kWheelDiamterMeters);

        if (Math.abs(velocitySetpoint) > deadzone) {
            steerMotor.setSetpoint(optimizedState.angle.getRadians());
            driveMotor.getTalonFX().setControl(new VelocityDutyCycle(velocitySetpoint).withEnableFOC(true));
        } else {
            steerMotor.setRawPercentage(0);
        }
    }

    public void setDeadzone(double deadzone) {
        this.deadzone = deadzone;
    }

}
