package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SteelTalonsLogger;
import frc.robot.util.SmaxProfiles.SteelTalonsSparkMaxBangBang;
import frc.robot.util.SmaxProfiles.SteelTalonsSparkMaxFlywheel;
import frc.robot.util.SmaxProfiles.SteelTalonsSparkMaxServo;

public class Shooter extends SubsystemBase {

  private SteelTalonsSparkMaxBangBang leftFlywheel;
  private SteelTalonsSparkMaxBangBang rightFlywheel;
  private SteelTalonsSparkMaxFlywheel feeder;
  private SteelTalonsSparkMaxServo ampMotor;
  private SteelTalonsSparkMaxServo pivotMaster;
  private SteelTalonsSparkMaxServo pivotSlave;

  private double leftShooterSetpoint = 0.0;
  private double rightShooterSetpoint = 0.0;
  private double feederSetpoint = 0.0;
  private Rotation2d ampSetpoint = new Rotation2d();
  private Rotation2d pivotSetpoint = new Rotation2d();
  private boolean homing = false;

  private DigitalInput beamBreak;

  private static Shooter instance;

  /** Creates a new Shooter. */
  public Shooter() {
    ShooterConstants.configureShooter();

    leftFlywheel = new SteelTalonsSparkMaxBangBang(ShooterConstants.shooterLeftFlywheelConfig);
    rightFlywheel = new SteelTalonsSparkMaxBangBang(ShooterConstants.shooterRightFlywheelConfig);
    feeder = new SteelTalonsSparkMaxFlywheel(ShooterConstants.feederRollerConfig);
    ampMotor = new SteelTalonsSparkMaxServo(ShooterConstants.ampPivotConfig);

    pivotMaster = new SteelTalonsSparkMaxServo(ShooterConstants.shooterPivotConfig);
    pivotSlave = new SteelTalonsSparkMaxServo(ShooterConstants.shooterPivotConfig, ShooterConstants.SHOOTER_PIVOT_SLAVE_MOTOR_ID);
    pivotSlave.getSmax().follow(pivotMaster.getSmax(), true);
    pivotMaster.setPosition(0.0);

    beamBreak = new DigitalInput(ShooterConstants.BEAM_BREAKER_PORT);

    instance = this;
    
  }

  public static Shooter getInstance() {
    return instance;
  }

  public void setFlywheelSetpoint(double left, double right) {
    this.leftShooterSetpoint = left;
    this.rightShooterSetpoint = right;
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

  public boolean flywheelAtGoal() {
    return leftFlywheel.getError() < ShooterConstants.FLYWHEEL_TOLERANCE_RPM && leftFlywheel.getError() < ShooterConstants.FLYWHEEL_TOLERANCE_RPM;
  }

  public boolean pivotAtGoal() {
    return Math.abs(pivotMaster.getError()) < ShooterConstants.PIVOT_TOLERANCE_RAD.getRadians();
  }

  public boolean ampAtGoal() {
    return Math.abs(ampMotor.getError()) < ShooterConstants.AMP_TOLERANCE_RAD.getRadians();
  }

  public void hardSetPivot(double percent) {
    pivotMaster.setRaw(percent);
  }

  public void hardSetAmp(double percent) {
    ampMotor.setRaw(percent);
  }

  @Override
  public void periodic() {
    leftFlywheel.setSetpoint(this.leftShooterSetpoint, 0.0);
    rightFlywheel.setSetpoint(this.rightShooterSetpoint, 0.0);

    if (homing) {
      hardSetPivot(-0.05);
    } else {
      pivotMaster.setSetpoint(this.pivotSetpoint.getRadians(), 0.0);
    }

    ampMotor.setSetpoint(this.ampSetpoint.getRadians(), 0.0);
    feeder.setSetpoint(this.feederSetpoint, 0.0);

    log();
  }

  public SteelTalonsSparkMaxBangBang getLeftFlywheel() {
    return leftFlywheel;
  }

  public SteelTalonsSparkMaxBangBang getRightFlywheel() {
    return rightFlywheel;
  }

  public SteelTalonsSparkMaxServo getShooterPivot() {
    return pivotMaster;
  }

  public SteelTalonsSparkMaxServo getShooterAmp() {
    return ampMotor;
  }

  public SteelTalonsSparkMaxFlywheel getFeeder() {
    return feeder;
  }

  public boolean loaded() {
    return !beamBreak.get();
  }

  public boolean getHoming() {
    return homing;
  }

  public void setHoming(boolean homing) {
    this.homing = homing;
  }

  private void log() {
    SteelTalonsLogger.post("Shooter Left Speed", leftFlywheel.getVelocity());
    SteelTalonsLogger.post("Shooter Right Speed", rightFlywheel.getVelocity());
    SteelTalonsLogger.post("Feeder Speed", feeder.getVelocity());
    SteelTalonsLogger.post("Pivot Position", pivotMaster.getPosition());
    SteelTalonsLogger.post("Amp Position", ampMotor.getPosition());
  }

}

