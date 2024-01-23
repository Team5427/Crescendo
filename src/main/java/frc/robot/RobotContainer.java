// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.io.PilotingControls;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.util.AutonUtil;
import frc.robot.util.SteelTalonsLogger;
import frc.robot.util.Localization.SteelTalonsLocalization;

public class RobotContainer {

  private CommandXboxController joy;

  private SwerveDrivetrain drivetrain;
  // private Intake intake;
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    joy = new CommandXboxController(0);

    drivetrain = new SwerveDrivetrain();
    drivetrain.setDefaultCommand(drivetrain.getDriveCommand(joy));

    // intake = new Intake();
    // intake.setDefaultCommand(intake.getCommand(joy));

    new SteelTalonsLocalization(); //has to be after drivetrain
    new SteelTalonsLogger();
    // SteelTalonsLocalization.getInstance().resetPose(new Pose2d(1, 1, new Rotation2d()));
    new AutonUtil(); //has to be last

    autoChooser = AutoBuilder.buildAutoChooser();
    // autoChooser.addOption("WrkPlease", new PathPlannerAuto("Test"));
    // autoChooser.setDefaultOption("WrkPlease", new PathPlannerAuto("Test"));
    // autoChooser.addOption("MyAuto", new PathPlannerAuto("MyAuto"));
    // autoChooser.setDefaultOption("Test", new PathPlannerAuto("Test"));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    new PilotingControls(new CommandXboxController(0));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
