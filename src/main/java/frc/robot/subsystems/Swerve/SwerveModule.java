package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.MiscUtil;
import frc.robot.util.STSmaxConfig;
import frc.robot.util.SteelTalonsLogger;
import frc.robot.util.SmaxProfiles.SteelTalonsSparkMaxSimpleServo;

public class SwerveModule {
    private TalonFX driveMotor;
    // private SteelTalonsSparkMaxFlywheel driveMotor;
    private SteelTalonsSparkMaxSimpleServo steerMotor;
    private CANcoder canCoder;
    
    public SwerveModule(int talonID, TalonFXConfiguration driveConfig, STSmaxConfig steerConfig, int canCoderID, double offset) {

        driveMotor = new TalonFX(talonID);
        driveMotor.getConfigurator().apply(driveConfig);
        DrivetrainConstants.configureDriveTalon(driveMotor);

        steerMotor = new SteelTalonsSparkMaxSimpleServo(DrivetrainConstants.configureSteerNeo(steerConfig));

        canCoder = new CANcoder(canCoderID);
        DrivetrainConstants.configureCanCoder(canCoder, offset);

        driveMotor.setPosition(0);
        steerMotor.setPosition(canCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(MiscUtil.DTrotToMeters(driveMotor.getPosition().getValueAsDouble()), new Rotation2d(steerMotor.getPosition()));
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(MiscUtil.DTrotToMeters(driveMotor.getVelocity().getValueAsDouble()), new Rotation2d(steerMotor.getPosition()));
    }

    public void setModuleState(SwerveModuleState state) {
        SwerveModuleState newState = SwerveModuleState.optimize(state, canCoderRot());

        double velocitySetpoint = MiscUtil.DTmetersToRot(newState.speedMetersPerSecond);
        Rotation2d rotSetpoint = newState.angle;

        if (Math.abs(velocitySetpoint) > 0.05) {
            steerMotor.setSetpoint(rotSetpoint.getRadians(), 0);
            driveMotor.setControl(new VelocityVoltage(velocitySetpoint).withEnableFOC(true));
        } else {
            steerMotor.forceStop();
            driveMotor.stopMotor();
        }
    }

    public Rotation2d canCoderRot() {
        return Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetController() {
        steerMotor.resetController();
    } 

    public void log(String name) {
        SteelTalonsLogger.post(name + " velocity", getModuleState().speedMetersPerSecond);
        SteelTalonsLogger.post(name + " position", getModulePosition().distanceMeters);
        SteelTalonsLogger.post(name + " angle", getModuleState().angle.getRadians());
        SteelTalonsLogger.post(name + " absolute angle", canCoder.getAbsolutePosition().getValueAsDouble());
        // steerMotor.log();
    }
}
