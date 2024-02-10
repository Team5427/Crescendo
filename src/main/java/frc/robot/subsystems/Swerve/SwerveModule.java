package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.STSmaxConfig;
import frc.robot.util.SteelTalonsLogger;
import frc.robot.util.SteelTalonsSparkMaxFlywheel;
import frc.robot.util.SteelTalonsSparkMaxServo;

public class SwerveModule {
    // private TalonFX driveMotor;
    private SteelTalonsSparkMaxFlywheel driveMotor;
    private SteelTalonsSparkMaxServo steerMotor;
    private CANcoder canCoder;
    
    private SendableChooser<Double> deadzone;

    private SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

    public SwerveModule(STSmaxConfig driveConfig, STSmaxConfig steerConfig, int canCoderID, double offset) {
        DrivetrainConstants.configureMotors();

        driveMotor = new SteelTalonsSparkMaxFlywheel(DrivetrainConstants.configureDriveNeo(driveConfig));
    
        steerMotor = new SteelTalonsSparkMaxServo(DrivetrainConstants.configureSteerNeo(steerConfig));

        canCoder = new CANcoder(canCoderID);
        DrivetrainConstants.configureCanCoder(canCoder, offset);

        driveMotor.setPosition(0);
        steerMotor.setPosition(canCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI);

        xLimiter = new SlewRateLimiter(DrivetrainConstants.SLEW_RATE_LIMIT);
        thetaLimiter = new SlewRateLimiter(DrivetrainConstants.SLEW_RATE_LIMIT);

        deadzone = new SendableChooser<Double>();
        deadzone.setDefaultOption("Competition", DrivetrainConstants.THRESHOLD_STOPPING_M_S_COMPETITION);
        deadzone.addOption("Tuning", DrivetrainConstants.THRESHOLD_STOPPING_M_S_TUNING);
        SmartDashboard.putData("Deadzone", deadzone);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveMotor.getPosition(), new Rotation2d(steerMotor.getPosition()));
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(driveMotor.getVelocity(), new Rotation2d(steerMotor.getPosition()));
    }

    public void setModuleState(SwerveModuleState state) {
        SwerveModuleState newState = SwerveModuleState.optimize(state, canCoderRot());
        // SwerveModuleState.optimize(state, null)
        // SwerveModuleState newState = state;

        double velocitySetpoint = newState.speedMetersPerSecond;
        Rotation2d rotSetpoint = newState.angle;

        // double velocitySetpoint = xLimiter.calculate(newState.speedMetersPerSecond);
        // double rotSetpoint = thetaLimiter.calculate(newState.angle.getRadians());
        if (Math.abs(velocitySetpoint) > deadzone.getSelected()) {
            steerMotor.setSetpoint(rotSetpoint.getRadians(), 0);
            // steerMotor.setSetpoint(rotSetpoint, 0);
            driveMotor.setSetpoint(velocitySetpoint, 0);
        } else {
            steerMotor.forceStop();
            driveMotor.forceStop();
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
