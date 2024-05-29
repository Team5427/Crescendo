package team5427.frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.drivers.CANDeviceId;
import lib.drivers.SteelTalonsLogger;
import lib.motors.MotorConfiguration;
import lib.motors.ProfiledSparkMax;
import lib.motors.SimpleSparkMax;
import lib.motors.SteelTalonFX;
import team5427.frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private SimpleSparkMax topMotor;
    private SimpleSparkMax bottomMotor;
    private SimpleSparkMax feeder;
    private SimpleSparkMax ampStick;
    private ProfiledSparkMax pivotLeader;
    private ProfiledSparkMax pivotFollower;

    public double topShooterSetpoint = 0.0;
    public double bottomShooterSetpoint = 0.0; // Maybe 1 setpoint only?
    public double feederSetpoint = 0.0;
    public double pivotSetpoint = 0.0;
    public double ampSetpoint = 0.0;
    public boolean isHomingPivot = false;
    public boolean isHomingAmp = false;

    private DigitalInput earlyBeamBrake;
    private DigitalInput lateBeamBrake;

    public boolean disabled; // For Goblin Mode

    private static Shooter instance;

    private Shooter() {
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

        // ShooterConstants.createConfigs();

        topMotor.apply(ShooterConstants.kTopMotorConfiguration);
        bottomMotor.apply(ShooterConstants.kBottomMotorConfiguration);
        feeder.apply(ShooterConstants.kFeederMotorConfiguration);
        ampStick.apply(ShooterConstants.kAmpMotorConfiguration);
        pivotLeader.apply(ShooterConstants.kPivotLeaderMotorConfiguration);
        pivotFollower.apply(ShooterConstants.kPivotFollowerMotorConfiguration);

    }

    @Override
    public void periodic() {
        if (disabled) {
            pivotLeader.setSetpoint(0);
            feeder.setSetpoint(0);
            ampStick.setSetpoint(0);
            topMotor.setSetpoint(0);
            bottomMotor.setSetpoint(0);
        } else {
            pivotLeader.setSetpoint(pivotSetpoint);
            feeder.setSetpoint(feederSetpoint);
            ampStick.setSetpoint(ampSetpoint);
            topMotor.setSetpoint(topShooterSetpoint);
            bottomMotor.setSetpoint(bottomShooterSetpoint);
        }
    }

    public void setPivot(Rotation2d setpoint) {
        pivotSetpoint = setpoint.getRadians();
    }

    public void moveFeeder(double setpoint) {
        feederSetpoint = setpoint;
    }

    public void moveAmp(Rotation2d setpoint) {
        ampSetpoint = setpoint.getRadians();
    }

    public void movePivot(double speed) {
        pivotLeader.setRawPercentage(speed);
    }

    public void moveAmp(double speed) {
        ampStick.setRawPercentage(speed);
    }

    public void moveFlywheels(double topSetpoint, double bottomSetpoint) {
        topShooterSetpoint = topSetpoint;
        bottomShooterSetpoint = bottomSetpoint;
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
        if (pivotSetpoint == ShooterConstants.kStow.getRadians()
                && Math.abs(pivotLeader.getError()) < 0.05) {
            return true;
        }
        return false;
    }

    public boolean isHandoffing() {
        if (Math.abs(pivotLeader.getError()) < 0.05 && pivotSetpoint == ShooterConstants.kHandoff.getRadians()) {
            return true;
        }
        return false;
    }

    public boolean pivotAtGoal() {
        if (Math.abs(pivotLeader.getError()) < 0.05) {
            return true;
        }
        return false;
    }

    public boolean pivotVelocityAtGoal() {
        if (Math.abs(pivotLeader.getEncoderVelocity()) < Units.degreesToRadians(0.05)) {
            return true;
        }
        return false;
    }

    public boolean ampAtGoal() {
        if (Math.abs(ampStick.getError()) < 0.05) {
            return true;
        }
        return false;
    }

    public boolean isAmping() {
        if (pivotLeader.getEncoderPosition() == ShooterConstants.kShootAmp.getRadians() && pivotAtGoal()
                && ampAtGoal()) {
            return true;
        }
        return false;
    }

    public boolean isLoaded() {
        return (!earlyBeamBrake.get() && !lateBeamBrake.get());
    }

    public boolean isPartiallyLoaded() {
        return (!earlyBeamBrake.get() && lateBeamBrake.get());
    }

    public boolean brokenBeamBreak() { // might not use this
        if (lateBeamBrake.get() == false && earlyBeamBrake.get() == true) {
            return true;
        }
        return false;
    }

    public void home() {
        pivotLeader.setEncoderPosition(0.0);
        pivotFollower.setEncoderPosition(0.0);
        ampStick.setEncoderPosition(0.0);
        feeder.setEncoderPosition(0.0);
        topMotor.setEncoderPosition(0.0);
        bottomMotor.setEncoderPosition(0.0);
    }

    public ProfiledSparkMax getPivot() {
        return pivotLeader;
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
            return instance;
        } else {
            return instance;
        }
    }
}
