package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SteelTalonsSparkMaxFlywheel;
import frc.robot.util.SteelTalonsSparkMaxServo;

public class Shooter extends SubsystemBase {

  private SteelTalonsSparkMaxFlywheel leftFlywheel;
  private SteelTalonsSparkMaxFlywheel rightFlywheel;
  private SteelTalonsSparkMaxFlywheel feeder;
  private SteelTalonsSparkMaxServo ampMotor;
  private SteelTalonsSparkMaxServo pivotMaster;
  private SteelTalonsSparkMaxServo pivotSlave;

  private double flywheelSetpoint;
  private double feederSetpoint;
  private Rotation2d ampSetpoint;
  private Rotation2d pivotSetpoint;

  private static Shooter instance;

  /** Creates a new Shooter. */
  public Shooter() {
    ShooterConstants.configureShooter();

    leftFlywheel = new SteelTalonsSparkMaxFlywheel(ShooterConstants.shooterLeftFlywheelConfig);
    rightFlywheel = new SteelTalonsSparkMaxFlywheel(ShooterConstants.shooterRightFlywheelConfig);
    feeder = new SteelTalonsSparkMaxFlywheel(ShooterConstants.feederRollerConfig);
    ampMotor = new SteelTalonsSparkMaxServo(ShooterConstants.ampPivotConfig);

    pivotMaster = new SteelTalonsSparkMaxServo(ShooterConstants.shooterPivotConfig);
    pivotSlave = new SteelTalonsSparkMaxServo(ShooterConstants.shooterPivotConfig);
    pivotSlave.getSmax().follow(pivotMaster.getSmax());

    instance = this;
    
  }

  public static Shooter getInstance() {
    return instance;
  }

  public void setFlywheelSetpoint(double setpoint) {
    flywheelSetpoint = setpoint;
  }

  public void setFeederSetpoint(double setpoint) {
    feederSetpoint = setpoint;
  }

  public void setAmpSetpoint(Rotation2d setpoint) {
    ampSetpoint = setpoint;
  }

  public void setPivotSetpoint(Rotation2d setpoint) {
    pivotSetpoint = setpoint;
  }

  public void targetSpeaker() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}

