package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Swerve.DriveCommand;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;

public class AutoAlignIntake {
    public Pose2d notePose2d;
    public Pose2d robotPose2d;
    public DriveCommand driveCommand;
    public SwerveDrivetrain swerve;
    public ChassisSpeeds robotSpeed;

    public AutoAlignIntake(Pose2d notePose2d, Pose2d robotPose2d, ChassisSpeeds robotSpeed, DriveCommand driveCommand, SwerveDrivetrain swerve){
        this.notePose2d = notePose2d;   
        this.robotPose2d = robotPose2d;
        this.robotSpeed = robotSpeed;
        this.driveCommand = driveCommand;
        this.swerve = swerve;
    }

    public int noteOutOfBound(){
        if(notePose2d.getTranslation().getAngle().getRadians() >= Units.degreesToRadians(180) && notePose2d.getTranslation().getAngle().getRadians() < Units.degreesToRadians(270)){
            System.out.println("180 - 270");
            return -1;
        } else if(notePose2d.getTranslation().getAngle().getRadians() >= Units.degreesToRadians(270) && notePose2d.getTranslation().getAngle().getRadians() < Units.degreesToRadians(360)){ // might be wrong??
            System.out.println("270 - 360");
            return 1;
        } else{
            return 0;
        }
    }

    public double calculateAngleMovement(double limelightAngle){
        
    }
    public boolean moveRobot(){
        int posValue = noteOutOfBound();
        double posAngle = notePose2d.getTranslation().getAngle().getRadians();
        if(posValue == 0){
            ChassisSpeeds speed = new ChassisSpeeds(1, 0, posAngle);
            driveCommand.executeAutoAlign(speed);
        } else if(posValue == 1){
            ChassisSpeeds speed = new ChassisSpeeds(1,0,posAngle);
            driveCommand.executeAutoAlign(speed);
        } else if(posValue == 2){
            ChassisSpeeds speed = new ChassisSpeeds(1,0,posAngle);
            driveCommand.executeAutoAlign(speed);
        }
        return false;
    }
}
