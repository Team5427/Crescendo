package frc.robot.subsystems.swerve;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.lib.kinematics.ConstrainedChassisSpeeds;
import frc.robot.lib.motors.MotorConfiguration;

public class SwerveDrivetrain extends SubsystemBase implements SwerveDeviceConfigs {
    public static SwerveDrivetrain instance;
    private List<SwerveModule> modules;
    private Pigeon2 gyro;
    private ConstrainedChassisSpeeds constrainedSetPoint;
    private ProfiledPIDController rotationalController;

    private double lastTime;

    public boolean isGoblin = false;

    public static SwerveDrivetrain getInstance() {
        if (instance == null) {
            instance = new SwerveDrivetrain();
            return instance;
        } else {
            return instance;
        }
    }

    private SwerveDrivetrain() {
        gyro = new Pigeon2(DriveTrainConstants.kPidgeonID.getDeviceNumber(),
                DriveTrainConstants.kPidgeonID.getBus());
        modules = new ArrayList<SwerveModule>();
        for (int i = 0; i < DriveTrainConstants.kDriveMotorIds.length; i++) {
            CANcoder encoder = new CANcoder(DriveTrainConstants.kCancoderIds[i].getDeviceNumber(),
                    DriveTrainConstants.kCancoderIds[i].getBus());
            encoder.getConfigurator().apply(DriveTrainConstants.kCanCoderConfigurations[i]);
            modules.add(new SwerveModule(DriveTrainConstants.kDriveMotorConfigurations[i],
                    DriveTrainConstants.kDriveMotorIds[i], DriveTrainConstants.kSteerMotorConfigurations[i],
                    DriveTrainConstants.kSteerMotorIds[i], encoder));
        }

        rotationalController = new ProfiledPIDController(8.0, 0, 0.25, new Constraints(
                DriveTrainConstants.kMaxPossibleSpeedMetersPerSecond,
                DriveTrainConstants.kTeleopMaxRotationalSpeed));
        rotationalController.enableContinuousInput(-Math.PI, Math.PI);

        lastTime = Timer.getFPGATimestamp();
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public void resetGyro(Rotation2d rotation) {
        gyro.setYaw(rotation.getDegrees());
    }

    public void setConstrainedChassisSpeeds(ConstrainedChassisSpeeds speeds) {
        constrainedSetPoint = speeds;
    }

    public ConstrainedChassisSpeeds getConstrainedChassisSpeeds() {
        return constrainedSetPoint;
    }

    public SwerveDriveWheelPositions getWheelPositions() {
        return new SwerveDriveWheelPositions(new SwerveModulePosition[] {
                modules.get(DriveTrainConstants.kFrontLeftModuleIdx).getModulePosition(),
                modules.get(DriveTrainConstants.kFrontRightModuleIdx).getModulePosition(),
                modules.get(DriveTrainConstants.kRearLeftModuleIdx).getModulePosition(),
                modules.get(DriveTrainConstants.kRearRightModuleIdx).getModulePosition()
        });
    }

    public void setDeadzone(double deadzone) {
        modules.forEach((m) -> {
            m.setDeadzone(deadzone);
        });
    }

    public void adjustConstrainedChassisSpeeds(ConstrainedChassisSpeeds adjustment) {
        constrainedSetPoint.adjustConstrainedChassisSpeeds(adjustment);
    }

    public ChassisSpeeds getVelocityVector() {
        SwerveModuleState[] states = new SwerveModuleState[modules.size()];
        for (int i = 0; i < modules.size(); i++) {
            states[i] = modules.get(i).getModuleState();
        }
        ChassisSpeeds speeds = DriveTrainConstants.kKinematics.toChassisSpeeds(states);
        return speeds;
    }

    public ChassisSpeeds getUnconstrainedDriveSpeeds(CommandXboxController controller) { // Confused on how to implement
        double[] cv = {
                -controller.getRightY() * DriveTrainConstants.kTeleopMaxTranslationalSpeed,
                -controller.getRightX() * DriveTrainConstants.kTeleopMaxTranslationalSpeed,
                Math.copySign(Math.pow(Math.abs(controller.getLeftX()), 2), -controller.getLeftX())
                        * DriveTrainConstants.kTeleopMaxRotationalSpeed,
                -controller.getLeftX() * DriveTrainConstants.kTeleopMaxRotationalSpeed
        };

        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(cv[0], cv[1], cv[2], Rotation2d.fromRotations(cv[3]));
        } else {

        }
        return new ChassisSpeeds();

    }

    public ConstrainedChassisSpeeds getConstrainedDriveSpeeds(CommandXboxController controller) { // Confused on how to
                                                                                                  // implement
        return new ConstrainedChassisSpeeds();
    }

    @Override
    public void periodic() {

    }
}
