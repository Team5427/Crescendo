package frc.robot.subsystems.Swerve;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.MiscUtil;
import frc.robot.util.SteelTalonsLogger;

public class SwerveDrivetrain extends SubsystemBase {
    
    public static SwerveDrivetrain instance;
    private Pigeon2 gyro;
    private List<SwerveModule> modules;
    private ChassisSpeeds setPoint = new ChassisSpeeds(); // X: m/s - Y: m/s - Theta: rad/s

    public SwerveDrivetrain() {
        instance = this;
        gyro = new Pigeon2(DrivetrainConstants.PIGEON_CAN_ID);
        gyro.reset();
        modules = List.of(
            new SwerveModule(
                DrivetrainConstants.FRONT_LEFT_DRIVE, 
                DrivetrainConstants.FRONT_LEFT_STEER, 
                DrivetrainConstants.FRONT_LEFT_CANCODER_ID,
                DrivetrainConstants.FRONT_LEFT_OFFSET
            ), //FRONT LEFT
            new SwerveModule(
                DrivetrainConstants.FRONT_RIGHT_DRIVE, 
                DrivetrainConstants.FRONT_RIGHT_STEER, 
                DrivetrainConstants.FRONT_RIGHT_CANCODER_ID,
                DrivetrainConstants.FRONT_RIGHT_OFFSET
            ), //FRONT RIGHT
            new SwerveModule(
                DrivetrainConstants.BACK_LEFT_DRIVE, 
                DrivetrainConstants.BACK_LEFT_STEER, 
                DrivetrainConstants.BACK_LEFT_CANCODER_ID,
                DrivetrainConstants.BACK_LEFT_OFFSET
            ), //BACK LEFT
            new SwerveModule(
                DrivetrainConstants.BACK_RIGHT_DRIVE, 
                DrivetrainConstants.BACK_RIGHT_STEER, 
                DrivetrainConstants.BACK_RIGHT_CANCODER_ID,
                DrivetrainConstants.BACK_RIGHT_OFFSET
            ) //BACK RIGHT
        );

    }

    public static SwerveDrivetrain getInstance() {
        return instance;
    }

    public Rotation2d getRotation() {
        return gyro.getRotation2d();
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

    @Override
    public void periodic() {
        if (DriverStation.isTeleop()) {
            SwerveModuleState[] states = DrivetrainConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(ChassisSpeeds.discretize(setPoint, Units.millisecondsToSeconds(20)));
            SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.MAX_PHYSICAL_SPEED_M_S); //FIXME
            for (int i = 0; i < modules.size(); i++) {
                modules.get(i).setModuleState(states[i]);
            }
        }

        if (DriverStation.isDisabled()) {
            for (int i = 0; i < modules.size(); i++) {
                modules.get(i).resetController();
            }
        }
        log();
    }

    public void setStatesAuton(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.MAX_PHYSICAL_SPEED_M_S); //FIXME
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).setModuleState(states[i]);
        }
    }

    public void setSpeedsAuton(ChassisSpeeds speeds) {
        SwerveModuleState[] states = DrivetrainConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, Units.millisecondsToSeconds(20)));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.MAX_PHYSICAL_SPEED_M_S); //FIXME
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).setModuleState(states[i]);
        }
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
            -controller.getLeftX() * DrivetrainConstants.MAX_ROTATION_SPEED_RAD_S_TELEOP
        }; //FIXME may need to negate

        if (!MiscUtil.isBlue()) {
            cv[0] = -cv[0];
            cv[1] = -cv[1];
        }

        return ChassisSpeeds.fromFieldRelativeSpeeds(cv[0], cv[1], cv[2], this.getRotation());
    }

    public void log() {
        SteelTalonsLogger.post("Drivetrain Setpoint X", setPoint.vxMetersPerSecond);
        SteelTalonsLogger.post("Drivetrain Setpoint Y", setPoint.vyMetersPerSecond);
        SteelTalonsLogger.post("Drivetrain Setpoint Theta", setPoint.omegaRadiansPerSecond);
        SteelTalonsLogger.post("Drivetrain Velocity X", getVelocityVector().vxMetersPerSecond);
        SteelTalonsLogger.post("Drivetrain Velocity Y", getVelocityVector().vyMetersPerSecond);
        SteelTalonsLogger.post("Drivetrain Velocity Theta", getVelocityVector().omegaRadiansPerSecond);

        modules.get(0).log("FRONT LEFT");
        // modules.get(1).log("FRONT RIGHT");
        // modules.get(2).log("BACK LEFT");
        // modules.get(3).log("BACK RIGHT");
    }

    public Command getDriveCommand() {
        return new DriveCommand(getInstance(), new CommandXboxController(0));
    }


}
