// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.io.OperatingControls;
import frc.robot.io.PilotingControls;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.BumpFeederIn;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.TargetSpeaker;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.Vision.ObjectDetector;
import frc.robot.subsystems.managing.AutonShoot;
import frc.robot.subsystems.managing.SubsystemManager;
import frc.robot.util.AutonUtil;
import frc.robot.util.LEDManager;
import frc.robot.util.SteelTalonsLogger;
import frc.robot.util.Localization.SteelTalonsLocalization;

public class RobotContainer {

  private SwerveDrivetrain drivetrain;
  private Intake intake;
  private SendableChooser<Command> autoChooser;
  private Shooter shooter;
  private LEDManager leds;
  private Climber climber;

  private static DigitalInput input;
  private Trigger zeroButton;

  private static ObjectDetector noteCam;
  private static ObjectDetector tagCam;

  public RobotContainer() {

    drivetrain = new SwerveDrivetrain();
    drivetrain.setDefaultCommand(drivetrain.getDriveCommand(new CommandXboxController(0)));

    intake = new Intake();
    shooter = new Shooter();

    noteCam = new ObjectDetector("limelight-notecam"); // may need to move into intake subsystem
    tagCam = new ObjectDetector("limelight-front"); //change to limelight-shooter

    leds = new LEDManager();

    climber = new Climber();

    input = new DigitalInput(3);

    registerNamedCommands(); // Register commands BEFORE any other auton shenanigans

    new SteelTalonsLocalization(); // has to be after drivetrain
    new SteelTalonsLogger();
    new AutonUtil(); // has to be last

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    new PilotingControls(new CommandXboxController(0));
    new OperatingControls(new CommandXboxController(1));
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Complex Intake", SubsystemManager.getComplexIntakeCommand(null));
    NamedCommands.registerCommand("Auton Shoot", new SequentialCommandGroup(
      // new BumpFeederIn().withTimeout(1.0),
      new WaitUntilCommand(Shooter.getInstance()::atStow).withTimeout(1.0),
      new TargetSpeaker()
    ));
    NamedCommands.registerCommand("Cum And Go", SubsystemManager.cumAndGo());
  }

  public static ObjectDetector getNoteCam() {
    return noteCam;
  }

  public static ObjectDetector getTagCam() {
    return tagCam;
  }

  public static DigitalInput getLimitSwitch() {
    return input;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
