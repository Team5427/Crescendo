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
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.DrivetrainConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.util.MiscUtil;
import frc.robot.util.SteelTalonsLogger;

public class SteelTalonsLocalization extends SubsystemBase {

    private static SteelTalonsLocalization instance;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private ArrayList<ApriltagCam> camList;

    private final String leftCamName = "left";
    private final String rightCamName = "right";
    private final Transform3d leftRobotToCam = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    private final Transform3d rightRobotToCam = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));

    private SwerveDrivePoseEstimator poseEstimator;

    private Field2d field;

    public SteelTalonsLocalization() {
        instance = this;
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException ex) {
            System.err.println("Couldn't load field");
        }

        // camList.add(new ApriltagCam(leftCamName, leftRobotToCam, aprilTagFieldLayout));
        // camList.add(new ApriltagCam(rightCamName, rightRobotToCam, aprilTagFieldLayout));

        poseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.SWERVE_DRIVE_KINEMATICS, 
            SwerveDrivetrain.getInstance().getRotation(), 
            SwerveDrivetrain.getInstance().getWheelPositions().positions, 
            new Pose2d(), 
            VecBuilder.fill(0.5, 0.5, 0), 
            VecBuilder.fill(0.03, 0.03, Double.MAX_VALUE)
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
        Pose2d refPose = poseEstimator.getEstimatedPosition();

        field.setRobotPose(getPose());
        SteelTalonsLogger.postComplex("Field 2d", field);

        if (camList != null) {
            for (ApriltagCam cam : camList) {
                Optional<SteelTalonsVisionMeasurement> estimate = LocalizationUtil.findConfidence(cam.getUpdate(refPose), refPose);
                if (estimate.isPresent()) {
                    SteelTalonsVisionMeasurement m = estimate.get();
                    poseEstimator.addVisionMeasurement(m.getPose(), m.getTimestamp(), m.getConfidence());
                } 
            }

        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d newPose) {
        // Pose2d resetPose = MiscUtil.isBlue() ? newPose : MiscUtil.flip(newPose);
        // SteelTalonsLogger.post("ran reset pose", true);
        // System.err.println("ran method");
        poseEstimator.resetPosition(
            SwerveDrivetrain.getInstance().getRotation(), 
            SwerveDrivetrain.getInstance().getWheelPositions().positions, 
            newPose
        );
    }

}
