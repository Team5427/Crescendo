// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.io.OperatingControls;
import frc.robot.io.PilotingControls;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.BumpFeeder;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.Vision.ObjectDetector;
import frc.robot.subsystems.managing.SubsystemManager;
import frc.robot.util.AutonUtil;
import frc.robot.util.SteelTalonsLogger;
import frc.robot.util.Localization.SteelTalonsLocalization;

public class RobotContainer {

  private SwerveDrivetrain drivetrain;
  private Intake intake;
  private SendableChooser<Command> autoChooser;
  private Shooter shooter;

  private static ObjectDetector noteCam;
  private static ObjectDetector tagCam;

  public RobotContainer() {

    drivetrain = new SwerveDrivetrain();
    drivetrain.setDefaultCommand(drivetrain.getDriveCommand(new CommandXboxController(0)));

    intake = new Intake();
    shooter = new Shooter();

    noteCam = new ObjectDetector("limelight-notecam"); // may need to move into intake subsystem
    tagCam = new ObjectDetector("limelight-front");

    registerNamedCommands(); // Register commands BEFORE any other auton shenanigans

    new SteelTalonsLocalization(); //has to be after drivetrain
    new SteelTalonsLogger();
    new AutonUtil(); //has to be last

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    new PilotingControls(new CommandXboxController(0));
    new OperatingControls(new CommandXboxController(1));
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Use Intake", Intake.getInstance().getIntakeCommand());
    NamedCommands.registerCommand("Eject Note", Intake.getInstance().getIntakeEjaculation());
    NamedCommands.registerCommand("Feed Note", new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                Shooter.getInstance().getShooterHandoff(), 
                new BumpFeeder(),
                new InstantCommand(() -> {
                    Shooter.getInstance().setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM, ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
                })
            ),
            Intake.getInstance().getIntakeHandoff()
        ));
    NamedCommands.registerCommand("Complex Intake", SubsystemManager.getComplexIntakeCommand());
    NamedCommands.registerCommand("Auton Static Shoot", SubsystemManager.autonStaticShootCommand());
  }

  public static ObjectDetector getNoteCam() {
    return noteCam;
  }

  public static ObjectDetector getTagCam() {
    return tagCam;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
