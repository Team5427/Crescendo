package frc.robot.util.Localization;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.DrivetrainConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.util.MiscUtil;
import frc.robot.util.SteelTalonsLogger;

public class SteelTalonsLocalization extends SubsystemBase {

    private static SteelTalonsLocalization instance;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private ArrayList<ApriltagCam> camList;
    private Optional<Pose2d> lastPose = Optional.empty();
    private Pose2d refPose = new Pose2d();

    private final String leftCamName = "leftcam";
    private final String rightCamName = "rightcam";
    private final Transform3d leftRobotToCam = new Transform3d(Units.inchesToMeters(-12.842), Units.inchesToMeters(11.992), Units.inchesToMeters(9.385), new Rotation3d(0, Math.toRadians(-35), Math.PI));
    private final Transform3d rightRobotToCam = new Transform3d(Units.inchesToMeters(-12.842), Units.inchesToMeters(-11.992), Units.inchesToMeters(9.385), new Rotation3d(0, Math.toRadians(-35), Math.PI));

    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveDriveOdometry poseOdometry; //used in auton only  - for intake pathing

    private Field2d field;

    public SteelTalonsLocalization() {
        instance = this;
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException ex) {
            System.err.println("Couldn't load field");
        }

        camList = new ArrayList<ApriltagCam>();

        camList.add(new ApriltagCam(leftCamName, leftRobotToCam, aprilTagFieldLayout));
        camList.add(new ApriltagCam(rightCamName, rightRobotToCam, aprilTagFieldLayout));

        poseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.SWERVE_DRIVE_KINEMATICS, 
            SwerveDrivetrain.getInstance().getRotation(), 
            SwerveDrivetrain.getInstance().getWheelPositions().positions, 
            new Pose2d(), 
            VecBuilder.fill(2.5, 2.5, 0.0), 
            VecBuilder.fill(0.03, 0.03, Double.MAX_VALUE)
        );

        poseOdometry = new SwerveDriveOdometry(
            DrivetrainConstants.SWERVE_DRIVE_KINEMATICS, 
            SwerveDrivetrain.getInstance().getRotation(), 
            SwerveDrivetrain.getInstance().getWheelPositions().positions, 
            new Pose2d()
        );

        field = new Field2d();
    }

    public static SteelTalonsLocalization getInstance() {
        return instance;
    }

    @Override
    public void periodic() {
        SwerveDriveWheelPositions dtWheelPositions = SwerveDrivetrain.getInstance().getWheelPositions();
        Rotation2d gyroAngle = SwerveDrivetrain.getInstance().getRotation();
        poseEstimator.update(gyroAngle, dtWheelPositions);
        if (DriverStation.isAutonomous()) {
            poseOdometry.update(gyroAngle, dtWheelPositions);
            field.setRobotPose(getOdometryPose());
        } else {
            field.setRobotPose(getPose());
        }

        refPose = poseEstimator.getEstimatedPosition();

        SteelTalonsLogger.postComplex("Field 2d", field);

        if (camList != null) {
            for (ApriltagCam cam : camList) {
                Optional<SteelTalonsVisionMeasurement> estimate = LocalizationUtil.findConfidence(cam.getUpdate(refPose), refPose, lastPose);
                if (estimate.isPresent()) {
                    SteelTalonsVisionMeasurement m = estimate.get();
                    SteelTalonsLogger.post("Vision measurement", String.valueOf(m.getConfidence().get(0, 0)) + " - " + Timer.getFPGATimestamp());
                    poseEstimator.addVisionMeasurement(m.getPose(), m.getTimestamp(), m.getConfidence());
                    lastPose = Optional.of(m.getPose());
                    field.getObject(cam.getName()).setPose(m.getPose());
                } else {
                    lastPose = Optional.empty();
                }
            }

        }

        SteelTalonsLogger.post("targeting information parallel", MiscUtil.targetingInformation()[0]);
        SteelTalonsLogger.post("targeting information perp", MiscUtil.targetingInformation()[1]);
        SteelTalonsLogger.post("targeting information distance", MiscUtil.targetingInformation()[2]);
        SteelTalonsLogger.post("targeting information angError", MiscUtil.targetingInformation()[3]);
        SteelTalonsLogger.post("targeting information translation Angle", MiscUtil.targetingInformation()[4]);
        SteelTalonsLogger.post("camera based targeting distance", RobotContainer.getTagCam().speakerDist());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getOdometryPose() {
        return poseOdometry.getPoseMeters();
    }

    public void resetPose(Pose2d newPose) {
        poseEstimator.resetPosition(
            SwerveDrivetrain.getInstance().getRotation(), 
            SwerveDrivetrain.getInstance().getWheelPositions().positions, 
            newPose
        );

        lastPose = Optional.empty();
        refPose = newPose;
    }

    public void resetOdometryPose(Pose2d newPose) {
        poseOdometry.resetPosition(
            SwerveDrivetrain.getInstance().getRotation(), 
            SwerveDrivetrain.getInstance().getWheelPositions().positions, 
            newPose
        );

        resetPose(newPose);
    }

    public void resetCameras() {
        for (ApriltagCam cam : camList) {
            cam.resetCamera();
        }
    }

    public Translation2d translationFromSpeaker() {
        return MiscUtil.isBlue() ? getPose().getTranslation().minus(MiscUtil.speaker_Pose.getTranslation()) : getPose().getTranslation().minus(MiscUtil.flip(MiscUtil.speaker_Pose.getTranslation()));
    }
}
