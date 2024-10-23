// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
// import frc.robot.subsystems.Shooter.HomeAmp;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.managing.SubsystemManager;
import frc.robot.util.LEDManager;
import frc.robot.util.SteelTalonsLogger;
import frc.robot.util.LEDManager.LEDState;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;


  private RobotContainer m_robotContainer;
  private DigitalInput zeroButton;


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    FollowPathCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    LEDManager.getInstance().setState(LEDState.kDisabled);
  }

  @Override
  public void disabledPeriodic() {
    SteelTalonsLogger.post("zero button", !RobotContainer.getLimitSwitch().get());
    if (RobotContainer.getLimitSwitch().get()) {
      LEDManager.getInstance().setState(LEDState.kEmpty);
      Shooter.getInstance().getShooterPivot().setPosition(new Rotation2d(-.0274).getRadians());
      Intake.getInstance().getPivot().setPosition(IntakeConstants.HARDSTOP_POS.getRadians());
      // Shooter.getInstance().getShooterAmp().setPosition(ShooterConstants.AMP_HARDSTOP.getRadians());
    } else {
      LEDManager.getInstance().setState(LEDState.kDisabled);
    }
  }

  @Override
  public void disabledExit() {
    LEDManager.getInstance().resetStates();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // CommandScheduler.getInstance().schedule(new HomeAmp());
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
