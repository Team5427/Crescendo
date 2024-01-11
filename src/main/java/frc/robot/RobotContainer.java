// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.io.PilotingControls;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.util.AutonUtil;
import frc.robot.util.SteelTalonsLogger;
import frc.robot.util.Localization.SteelTalonsLocalization;

public class RobotContainer {

  private SwerveDrivetrain drivetrain;
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    drivetrain = new SwerveDrivetrain();
    drivetrain.setDefaultCommand(drivetrain.getDriveCommand());

    new SteelTalonsLocalization(); //has to be after drivetrain
    new SteelTalonsLogger();
    new AutonUtil(); //has to be last

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    new PilotingControls(new CommandXboxController(0));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
