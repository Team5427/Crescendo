package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.lib.motors.ProfiledSparkMax;
import frc.robot.lib.motors.SteelTalonFX;

public class Intake extends SubsystemBase {
    private ProfiledSparkMax pivotMotor;
    private SteelTalonFX intakeMotor;
    private DigitalInput beamBreak;

    private Rotation2d setpoint;
    private double rollerSpeed; // in Meters per Second

    private Intake instance;

    public boolean disabled = false; // for goblin mode

    public Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
            return instance;
        } else {
            return instance;
        }
    }

    private Intake() {
        pivotMotor = new ProfiledSparkMax(IntakeConstants.kPivotMotorID);
        intakeMotor = new SteelTalonFX(IntakeConstants.kIntakeMotorID);
        beamBreak = new DigitalInput(IntakeConstants.kBeamBreakPort);
        pivotMotor.apply(IntakeConstants.kPivotMotorConfiguration);
        intakeMotor.apply(IntakeConstants.kintakeMotorConfiguration);
        setpoint = IntakeConstants.kHome;
        rollerSpeed = 0.0;
    }

    @Override
    public void periodic() {
        if (disabled) {
            intakeMotor.setSetpoint(0);
            pivotMotor.setSetpoint(new Rotation2d(Units.degreesToRadians(0)));
        } else {
            intakeMotor.setSetpoint(rollerSpeed);
            pivotMotor.setSetpoint(setpoint);
        }
    }

    public SteelTalonFX getIntakeMotor() {
        return intakeMotor;
    }

    public ProfiledSparkMax getPivotMotor() {
        return pivotMotor;
    }

    public DigitalInput getBeamBreak() {
        return beamBreak;
    }

    public double getRollerSpeed() {
        return rollerSpeed;
    }

    public Rotation2d getPivotSetpoint() {
        return setpoint;
    }

    public boolean hasNote() {
        return !beamBreak.get();
    }

    public void setPivotSetpoint(Rotation2d setpoint) {
        this.setpoint = setpoint;
    }

    public void setRollerSetpoint(double speed) {
        this.rollerSpeed = speed;
    }

    public boolean isHandoffing() {
        return isHandoffPosition() && hasNote();
    }

    public boolean isIntaking() {
        return isIntakePosition() && !hasNote();
    }

    public boolean isStowed() {
        return isStowPosition() && !hasNote();
    }

    public boolean isHandoffPosition() {
        return setpoint == IntakeConstants.kHandoff && getPivotError() < 0.05;
    }

    public boolean isStowPosition() {
        return setpoint == IntakeConstants.kStow && getPivotError() < 0.05;
    }

    public boolean isIntakePosition() {
        return setpoint == IntakeConstants.kIntake && getPivotError() < 0.05;
    }

    public double getPivotError() {
        return Math.abs(Rotation2d.fromRotations(pivotMotor.getEncoderPosition()).minus(setpoint).getRadians());
    }

    public double getRollerError() {
        return Math.abs(intakeMotor.getEncoderVelocity() - rollerSpeed);
    }
}
