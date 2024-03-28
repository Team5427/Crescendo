package frc.robot.subsystems.Swerve;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.MiscUtil;
import frc.robot.util.SteelTalonsLogger;
import frc.robot.util.Localization.SteelTalonsLocalization;

public class SwerveDrivetrain extends SubsystemBase {
    
    public static SwerveDrivetrain instance;
    private Pigeon2 gyro;
    private List<SwerveModule> modules;
    private ChassisSpeeds setPoint = new ChassisSpeeds(); // X: m/s - Y: m/s - Theta: rad/s
    private ChassisSpeeds adjustment = new ChassisSpeeds();
    private DriveConfig driveConfig = DrivetrainConstants.DEFAULT_DRIVE_CONFIG;
    private ProfiledPIDController rotController;

    private double lastTime;

    public SwerveDrivetrain() {
        instance = this;
        gyro = new Pigeon2(DrivetrainConstants.PIGEON_CAN_ID);
        gyro.reset();
        DrivetrainConstants.configureMotors();
        modules = List.of(
            new SwerveModule(
                DrivetrainConstants.FRONT_LEFT_DRIVE_ID,
                DrivetrainConstants.FRONT_LEFT_DRIVE, 
                DrivetrainConstants.FRONT_LEFT_STEER, 
                DrivetrainConstants.FRONT_LEFT_CANCODER_ID,
                DrivetrainConstants.FRONT_LEFT_OFFSET
            ), //FRONT LEFT
            new SwerveModule(
                DrivetrainConstants.FRONT_RIGHT_DRIVE_ID,
                DrivetrainConstants.FRONT_RIGHT_DRIVE, 
                DrivetrainConstants.FRONT_RIGHT_STEER, 
                DrivetrainConstants.FRONT_RIGHT_CANCODER_ID,
                DrivetrainConstants.FRONT_RIGHT_OFFSET
            ), //FRONT RIGHT
            new SwerveModule(
                DrivetrainConstants.BACK_LEFT_DRIVE_ID,
                DrivetrainConstants.BACK_LEFT_DRIVE, 
                DrivetrainConstants.BACK_LEFT_STEER, 
                DrivetrainConstants.BACK_LEFT_CANCODER_ID,
                DrivetrainConstants.BACK_LEFT_OFFSET
            ), //BACK LEFT
            new SwerveModule(
                DrivetrainConstants.BACK_RIGHT_DRIVE_ID,
                DrivetrainConstants.BACK_RIGHT_DRIVE, 
                DrivetrainConstants.BACK_RIGHT_STEER, 
                DrivetrainConstants.BACK_RIGHT_CANCODER_ID,
                DrivetrainConstants.BACK_RIGHT_OFFSET
            ) //BACK RIGHT
        );

        rotController = new ProfiledPIDController(8.0, 0, 0.25, new Constraints(
            DrivetrainConstants.MAX_ROTATION_SPEED_RAD_S_TELEOP, 
            DrivetrainConstants.MAX_ROTATION_SPEED_RAD_S_TELEOP
        ));

        rotController.enableContinuousInput(-Math.PI, Math.PI);

        lastTime = Timer.getFPGATimestamp();

    }

    public static SwerveDrivetrain getInstance() {
        return instance;
    }

    public Rotation2d getRotation() {
        return gyro.getRotation2d();
    }

    public void resetGyro(Rotation2d rot) {
        gyro.setYaw(rot.getDegrees());
    }

    public void setDriveConfig(DriveConfig driveConfig) {
        this.driveConfig = driveConfig;
    }

    public SwerveDriveWheelPositions getWheelPositions() {
        return new SwerveDriveWheelPositions(new SwerveModulePosition[] {
            modules.get(0).getModulePosition(),
            modules.get(1).getModulePosition(),
            modules.get(2).getModulePosition(),
            modules.get(3).getModulePosition()
        });
    }

    public void setSetpoint(ChassisSpeeds speeds) {
        this.setPoint = speeds;
    }

    public ChassisSpeeds getSetpoint() {
        return this.setPoint;
    }

    public void setDeadzone(double deadzone) {
        modules.forEach((m) -> {
            m.setDeadzone(deadzone);
        });
    }

    public void adjustSpeeds(ChassisSpeeds adjustment) {
        this.adjustment = adjustment;
    }

    @Override
    public void periodic() {
        Optional<Rotation2d> rotLock = driveConfig.getAngleLock();

        setDeadzone(driveConfig.getDeadZone());

        ChassisSpeeds calculatedSetpoint = setPoint.times(driveConfig.getSpeedScalar());
        if (rotLock.isPresent()) {
            if (!MiscUtil.isBlue()) {
                rotLock = Optional.of(MiscUtil.flip(rotLock.get()));
            }
            calculatedSetpoint = new ChassisSpeeds(
                calculatedSetpoint.vxMetersPerSecond, 
                calculatedSetpoint.vyMetersPerSecond, 
                rotController.calculate(SteelTalonsLocalization.getInstance().getPose().getRotation().getRadians(), rotLock.get().getRadians())
            );
        } else {
            calculatedSetpoint = setPoint;
            rotController.reset(SteelTalonsLocalization.getInstance().getPose().getRotation().getRadians());
        }
        SwerveModuleState[] states = DrivetrainConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(ChassisSpeeds.discretize(calculatedSetpoint.plus(adjustment), Timer.getFPGATimestamp() - lastTime));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.MAX_PHYSICAL_SPEED_M_S);
        
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).setModuleState(states[i]);
        }

        if (DriverStation.isDisabled()) {
            for (int i = 0; i < modules.size(); i++) {
                modules.get(i).resetController();
            }
        } 

        log();

        lastTime = Timer.getFPGATimestamp();
    }

    public void setSpeedsAuton(ChassisSpeeds speeds) {
        setSetpoint(speeds);
    }

    public ChassisSpeeds getVelocityVector() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < modules.size(); i++) {
            states[i] = modules.get(i).getModuleState();
        }
        ChassisSpeeds speeds = DrivetrainConstants.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(states);
        return speeds;
    }

    public ChassisSpeeds getDriveSpeeds(CommandXboxController controller) {
        double[] cv = {
            -controller.getRightY() * DrivetrainConstants.MAX_TRANSLATION_SPEED_M_S_TELEOP, 
            -controller.getRightX() * DrivetrainConstants.MAX_TRANSLATION_SPEED_M_S_TELEOP, 
            Math.copySign(Math.pow(Math.abs(controller.getLeftX()), 2), -controller.getLeftX()) * DrivetrainConstants.MAX_ROTATION_SPEED_RAD_S_TELEOP,
            // -controller.getLeftX() * DrivetrainConstants.MAX_ROTATION_SPEED_RAD_S_TELEOP
        }; 

        double trigger = 1 - controller.getRightTriggerAxis();
        for (int i = 0; i  < cv.length; i++) {
            cv[i] = cv[i] * trigger;
        }
        
        return driveConfig.getFieldOp() ? ChassisSpeeds.fromFieldRelativeSpeeds(cv[0], cv[1], cv[2], MiscUtil.isBlue() ? SteelTalonsLocalization.getInstance().getPose().getRotation() : SteelTalonsLocalization.getInstance().getPose().getRotation().plus(Rotation2d.fromDegrees(180))) : new ChassisSpeeds(cv[0], cv[1], cv[2]);
    }

    public void log() {
        SteelTalonsLogger.post("Drivetrain Setpoint X", setPoint.vxMetersPerSecond);
        SteelTalonsLogger.post("Drivetrain Setpoint Y", setPoint.vyMetersPerSecond);
        SteelTalonsLogger.post("Drivetrain Setpoint Theta", setPoint.omegaRadiansPerSecond);
        SteelTalonsLogger.post("x speed", getVelocityVector().vxMetersPerSecond);
        SteelTalonsLogger.post("y speed", getVelocityVector().vyMetersPerSecond);
        SteelTalonsLogger.post("Delta Time", Timer.getFPGATimestamp() - lastTime);
        // modules.get(0).log("front left");
        // modules.get(1).log("front right");
        // modules.get(2).log("back left");
        // modules.get(3).log("back right");
    }

    public Command getDriveCommand(CommandXboxController joy) {
        return new DriveCommand(getInstance(), joy);
    }


}
