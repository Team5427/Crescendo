package frc.robot.subsystems.shooter;

import frc.robot.lib.motors.SteelTalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.drivers.CANDeviceId;
import frc.robot.lib.drivers.SteelTalonsLogger;
import frc.robot.lib.motors.MotorConfiguration;
import frc.robot.lib.motors.ProfiledSparkMax;
import frc.robot.lib.motors.SimpleSparkMax;

public class Shooter extends SubsystemBase {
    private SimpleSparkMax topMotor;
    private SimpleSparkMax bottomMotor;
    private SimpleSparkMax feeder;
    private SimpleSparkMax ampStick;
    private ProfiledSparkMax pivotLeader;
    private ProfiledSparkMax pivotFollower;

    private double topShooterSetpoint = 0.0;
    private double bottomShooterSetpoint = 0.0;
    private double feederSetpoint = 0.0;
    private boolean isHomingPivot = false;
    private boolean isHomingAmp = false;

    private DigitalInput earlyBeamBrake;
    private DigitalInput lateBeamBrake;

    public boolean disabled; // For Goblin Mode

    private static Shooter instance;

    public Shooter() {
        setName("Shooter");
        this.topMotor = new SimpleSparkMax(ShooterConstants.kTopMotorID);
        this.bottomMotor = new SimpleSparkMax(ShooterConstants.kBottomMotorID);
        this.feeder = new SimpleSparkMax(ShooterConstants.kFeederMotorID);
        this.ampStick = new SimpleSparkMax(ShooterConstants.kAmpMotorID);
        this.pivotLeader = new ProfiledSparkMax(ShooterConstants.kPivotLeaderMotorID);
        this.pivotFollower = new ProfiledSparkMax(ShooterConstants.kPivotFollowerMotorID);

        this.lateBeamBrake = new DigitalInput(ShooterConstants.kLateBeamBrakeID);
        this.earlyBeamBrake = new DigitalInput(ShooterConstants.kEarlyBeamBrakeID);

        disabled = false;

        pivotFollower.getSparkMax().follow(pivotLeader.getSparkMax());

        ShooterConstants.createConfigs();

        topMotor.apply(ShooterConstants.kTopMotorConfiguration);
        bottomMotor.apply(ShooterConstants.kBottomMotorConfiguration);
        feeder.apply(ShooterConstants.kFeederMotorConfiguration);
        ampStick.apply(ShooterConstants.kAmpMotorConfiguration);
        pivotLeader.apply(ShooterConstants.kPivotLeaderMotorConfiguration);
        pivotFollower.apply(ShooterConstants.kPivotFollowerMotorConfiguration);

        instance = this;

    }

    public void movePivot(Rotation2d setpoint) {
        pivotLeader.setSetpoint(setpoint);
    }

    public void moveFeeder(double setpoint) {
        feeder.setSetpoint(setpoint);
    }

    public void moveAmp(Rotation2d setpoint) {
        ampStick.setSetpoint(setpoint);
    }

    public void moveShooters(double topSetpoint, double bottomSetpoint) {
        topMotor.setSetpoint(topSetpoint);
        bottomMotor.setSetpoint(bottomSetpoint);
    }

    public double getAmpSetpoint() {
        return ampStick.getSetpoint();
    }

    public double getFeederSetpoint() {
        return feeder.getSetpoint();
    }

    public double getTopMotorSetpoint() {
        return topMotor.getSetpoint();
    }

    public double getBottomMotorSetpoint() {
        return bottomMotor.getSetpoint();
    }

    public double getPivotSetpoint() {
        return pivotLeader.getSetpoint();
    }

    public double getShootingVelocity() {
        return (topMotor.getEncoderVelocity() + bottomMotor.getEncoderVelocity()) / 2; // averages the speeds of the two
                                                                                       // rollers to give the overall
                                                                                       // shooting velocity
    }

    public boolean flywheelAtGoal() {
        return (Math.abs(topMotor.getError()) < ShooterConstants.kFlywheelTolerance)
                && (Math.abs(bottomMotor.getError()) < ShooterConstants.kFlywheelTolerance);
    }

    public boolean isStowed() {
        if (pivotLeader.getSetpoint() == ShooterConstants.kStow.getRotations() && pivotLeader.getError() < 0.05) {
            return true;
        }
        return false;
    }

    public boolean isHandoffing() {
        if (pivotLeader.getSetpoint() == ShooterConstants.kHandoff.getRotations() && pivotLeader.getError() < 0.05) {
            return true;
        }
        return false;
    }

    public boolean pivotAtGoal() {
        if (pivotLeader.getError() < 0.05) {
            return true;
        }
        return false;
    }

    public boolean isReadyToAmp() {
        if (pivotLeader.getSetpoint() == ShooterConstants.kShootAmp.getRotations() && pivotLeader.getError() < 0.05) {
            return true;
        }
        return false;
    }

    public Shooter getInstance() {
        return instance;
    }
}
