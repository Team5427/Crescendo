package frc.robot.subsystems.Shooter;

import java.sql.Driver;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.MiscUtil;
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
  private boolean homingPivot = false;
  private boolean homingAmp = false;

  private DigitalInput earlyBeamBrake;
  private DigitalInput lateBeamBrake;

  private static Shooter instance;

  /** Creates a new Shooter. */
  public Shooter() {
    ShooterConstants.configureShooter();

    leftFlywheel = new SteelTalonsSparkMaxBangBang(ShooterConstants.shooterLeftFlywheelConfig);
    rightFlywheel = new SteelTalonsSparkMaxBangBang(ShooterConstants.shooterRightFlywheelConfig);
    feeder = new SteelTalonsSparkMaxFlywheel(ShooterConstants.feederRollerConfig);
    feeder.disableLimiter();
    ampMotor = new SteelTalonsSparkMaxServo(ShooterConstants.ampPivotConfig);
    ampMotor.disableContinuousInput();

    pivotMaster = new SteelTalonsSparkMaxServo(ShooterConstants.shooterPivotConfig);
    pivotSlave = new SteelTalonsSparkMaxServo(ShooterConstants.shooterPivotConfig, ShooterConstants.SHOOTER_PIVOT_SLAVE_MOTOR_ID);
    pivotSlave.getSmax().follow(pivotMaster.getSmax(), true);
    pivotMaster.setPosition(0.0);

    earlyBeamBrake = new DigitalInput(ShooterConstants.EARLY_BEAM_BRAKER_PORT);
    lateBeamBrake = new DigitalInput(ShooterConstants.LATE_BEAM_BREAKER_PORT);

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
    // pivotMaster.resetController();
    pivotSetpoint = setpoint;
  }

  public void setShootingConfigSetpoints(ShootingConfiguration config) {
    setPivotSetpoint(config.getPivotAngle());
    setFlywheelSetpoint(config.getLeftSpeed(), config.getRightSpeed());
  } 

  public boolean flywheelAtGoal() {
    return Math.abs(rightFlywheel.getError()) < ShooterConstants.FLYWHEEL_TOLERANCE_RPM && Math.abs(leftFlywheel.getError()) < ShooterConstants.FLYWHEEL_TOLERANCE_RPM;
  }

  public boolean pivotAtGoal() {
    return pivotAtGoal(ShooterConstants.PIVOT_TOLERANCE_RAD.getRadians());
  }

  public boolean pivotAtGoal(double degTol) {
    return Math.abs(pivotMaster.getError()) < Units.degreesToRadians(degTol);
  }

  public boolean pivotAtVelGoal(double degTol) {
    return Math.abs(pivotMaster.getVelError()) < Units.degreesToRadians(degTol);
  }

  public boolean atHandoff() {
    return pivotAtGoal(1.0) && this.pivotSetpoint.equals(ShooterConstants.SHOOTER_PIVOT_HANDOFF);
  }

  public boolean atStow() {
    return pivotAtGoal(1.0) && this.pivotSetpoint.equals(ShooterConstants.SHOOTER_PIVOT_STOW);
  }

  public boolean notAtStow() {
    return !atStow();
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
    CommandXboxController tester = new CommandXboxController(1);
    if (MiscUtil.targetingInformation()[2] < 4.0 && loaded() && !tester.getHID().getBButton() && !tester.getHID().getYButton() && DriverStation.isTeleop()) {
      leftFlywheel.setSetpoint(ShooterConstants.FLYWHEEL_REV_SPEED_RPM, 0.0);
      rightFlywheel.setSetpoint(ShooterConstants.FLYWHEEL_REV_SPEED_RPM, 0.0);
  
    } else if (DriverStation.isAutonomous()) {
      leftFlywheel.setSetpoint(ShooterConstants.FLYWHEEL_AUTON_SPEED_RPM, 0.0);
      rightFlywheel.setSetpoint(ShooterConstants.FLYWHEEL_AUTON_SPEED_RPM, 0.0);

    } else {
      leftFlywheel.setSetpoint(this.leftShooterSetpoint, 0.0);
      rightFlywheel.setSetpoint(this.rightShooterSetpoint, 0.0);  
    }

    if (homingPivot) {
      hardSetPivot(0.05);
    } else {
      pivotMaster.setSetpoint(
        MathUtil.clamp(
          this.pivotSetpoint.getRadians(), 
          ShooterConstants.SHOOTER_PIVOT_MAX_ROT.getRadians(), 
          0.0
        ), 0.0);
    }

    if (homingAmp) {
      hardSetAmp(0.08);
    } else {
      ampMotor.setSetpoint(this.ampSetpoint.getRadians(), 0.0);
    }

    if (this.feederSetpoint == ShooterConstants.FEEDER_HOLD_SPEED && !inPosition() && loaded()) {
      feeder.setSetpoint(0.0, 0.0);
    } else {
      feeder.setSetpoint(this.feederSetpoint, 0.0);
    }

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
    return !earlyBeamBrake.get();
  }

  public boolean inPosition() {
    return loaded() && !lateBeamBrake.get();
  }

  public boolean getHoming() {
    return homingPivot;
  }

  public boolean getAmpHoming() {
    return homingAmp;
  }

  public void setHomingPivot(boolean homingPivot) {
    this.homingPivot = homingPivot;
  }

  public void setHomingAmp(boolean homingAmp) {
    this.homingAmp = homingAmp;
  }

  private void log() {
    SteelTalonsLogger.post("Shooter Left Speed", leftFlywheel.getVelocity());
    SteelTalonsLogger.post("Shooter Right Speed", rightFlywheel.getVelocity());
    // SteelTalonsLogger.post("Feeder Speed", feeder.getVelocity());
    SteelTalonsLogger.post("Pivot Position", pivotMaster.getPosition());
    SteelTalonsLogger.post("Pivot setpiint shooter", pivotMaster.getSetPoint());
    // SteelTalonsLogger.post("Amp Position", ampMotor.getPosition());
    SteelTalonsLogger.post("FLYWHEEL RAMPED", flywheelAtGoal());
    SteelTalonsLogger.post("Shooter Loaded", loaded());
    SteelTalonsLogger.post("Note Ready To Shoot", inPosition());
    // SteelTalonsLogger.post("top in position", !beamBreak.get());
    // SteelTalonsLogger.post("shooter flywheel at goal", flywheelAtGoal());
    // SteelTalonsLogger.post("pivot at goal", pivotAtGoal());
    SteelTalonsLogger.post("pivot at goal 3", pivotAtGoal(3));
  }

  public Command getHomingCommand() {
    return new HomeShooter();
  }

  public Command getShooterHandoff() {
    return new ShooterHandoff();
  }

  public Command getFeedCommand(double setpoint, Rotation2d pivotRot) {
    return new FeedShooter(setpoint, pivotRot, true);
  }

}

