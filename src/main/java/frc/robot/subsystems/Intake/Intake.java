package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.SteelTalonsLogger;
import frc.robot.util.SteelTalonsSparkMaxFlywheel;
import frc.robot.util.SteelTalonsSparkMaxServo;

public class Intake extends SubsystemBase {

    private SteelTalonsSparkMaxFlywheel roller;
    private TalonFX rollerTalon;
    private SteelTalonsSparkMaxServo pivot;
    private DigitalInput beamBreaker;
    private ArmFeedforward pivotFF;

    private double rollerSetpoint = IntakeConstants.INTAKE_SPEED_HOLD;
    private Rotation2d setpoint = new Rotation2d();

    private static Intake instance;

    private boolean isHoming;

    public Intake () {
        IntakeConstants.configureIntake();
        roller = new SteelTalonsSparkMaxFlywheel(IntakeConstants.ROLLER_CONFIG);
        rollerTalon = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 20;

        rollerTalon.getConfigurator().apply(config);
        pivot = new SteelTalonsSparkMaxServo(IntakeConstants.PIVOT_CONFIG);
        pivot.disableContinuousInput();
        resetPivotEncoder(IntakeConstants.HARDSTOP_POS);
        instance = this;

        pivotFF = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV, IntakeConstants.kA);
        beamBreaker = new DigitalInput(IntakeConstants.BEAM_BREAKER_PORT);
        isHoming = false;
    }

    public static Intake getInstance() {
        return instance;
    }

    public void setPivotSetpoint(Rotation2d setpoint) {
        this.setpoint = setpoint;
    }

    public void setRollerSetpoint(double speed) {
        this.rollerSetpoint = speed;       
    }

    public TalonFX getRoller() {
        return rollerTalon;
    }

    public SteelTalonsSparkMaxServo getPivot() {
        return pivot;
    }

    public void stopRoller() {
        rollerTalon.stopMotor();
    }

    public void hardSetPivot(double percent) {
        pivot.setRaw(percent);
    }

    public void hardSetRoller(double percent) {
        rollerTalon.set(percent);
    }

    public void resetPivotEncoder(Rotation2d resetPoint) {
        pivot.setPosition(resetPoint.getRadians());
    }

    public void setHoming(boolean homing) {
        isHoming = homing;
    }

    public boolean sensorCovered() {
        return !beamBreaker.get();
    }

    public boolean atGoal() {
        return Math.abs(pivot.getError()) < IntakeConstants.PIVOT_TOLERANCE_RAD;
    }

    public boolean getHoming() {
        return isHoming;
    }

    @Override
    public void periodic() {
        if (!isHoming) {
            pivot.setSetpoint(setpoint.getRadians(), 
            0.0
            // pivotFF.calculate(getPivot().getPosition(), pivot.getSetpointVelocity())
            );

            hardSetRoller(rollerSetpoint / IntakeConstants.MAX_KRAKEN_ROLLER_SPEED_M_S); //rollerSetpoint / IntakeConstants.MAX_KRAKEN_ROLLER_SPEED_M_S
        } else {
            hardSetPivot(0.05);
        }
        log();
    }

    public Command getBasicIntakeCommand() {
        return new IntakeCommand();
    }

    public Command getHomingCommand() {
        return new HomeIntake();
    }

    public Command getIntakeEjaculation() {
        return new TempIntakeEjaculation();
    }

    public void log() {
        SteelTalonsLogger.post("Intake pivot angle", pivot.getPosition());
        SteelTalonsLogger.post("Intake pivot setpoint", setpoint.getRadians());
        SteelTalonsLogger.post("Intake pivot error", pivot.getError());

        SteelTalonsLogger.post("Intake roller angle", roller.getVelocity());
        SteelTalonsLogger.post("Intake roller setpoint", this.rollerSetpoint);
        SteelTalonsLogger.post("Intake roller error", roller.getError());

        SteelTalonsLogger.post("setpoint velocity", pivot.getSetpointVelocity());

        SteelTalonsLogger.post("Beam Broken", beamBreaker.get());
    }

}