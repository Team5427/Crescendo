package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.FaultID;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.SteelTalonsLogger;
import frc.robot.util.SmaxProfiles.SteelTalonsSparkMaxServo;

public class Intake extends SubsystemBase {

    private TalonFX rollerTalon;
    private SteelTalonsSparkMaxServo pivot;
    private DigitalInput beamBreaker;

    private double rollerSetpoint = IntakeConstants.INTAKE_SPEED_HOLD;
    private Rotation2d setpoint = IntakeConstants.STOWED_POS;

    private static Intake instance;

    private boolean isHoming;

    private boolean isStalled;

    public Intake() {
        IntakeConstants.configureIntake();
        rollerTalon = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, "*");

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.SupplyCurrentThreshold = 70;
        config.CurrentLimits.SupplyTimeThreshold = 0.25;
        config.CurrentLimits.StatorCurrentLimit = 45;

        rollerTalon.getConfigurator().apply(config);
        pivot = new SteelTalonsSparkMaxServo(IntakeConstants.PIVOT_CONFIG);
        pivot.disableContinuousInput();
        resetPivotEncoder(IntakeConstants.HARDSTOP_POS);
        // pivot.resetController();
        // setPivotSetpoint(IntakeConstants.STOWED_POS);
        beamBreaker = new DigitalInput(IntakeConstants.BEAM_BREAKER_PORT);
        isHoming = false;

        instance = this;
    }

    public static Intake getInstance() {
        return instance;
    }

    public void setPivotSetpoint(Rotation2d setpoint) {
        // if (setpoint.equals(IntakeConstants.INTAKING_POS)) {
        // pivot.setAccel(IntakeConstants.ROLLER_CONFIG.maxAccel * 0.5);
        // } else {
        // pivot.setAccel(IntakeConstants.ROLLER_CONFIG.maxAccel);
        // }
        // pivot.resetController();
        // Timer.delay(0.15);
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

    public boolean isStalled() {
        return isStalled;
    }

    public void setLimits(int num) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 50;
        config.CurrentLimits.SupplyCurrentThreshold = 65;
        config.CurrentLimits.SupplyTimeThreshold = 0.25;
        config.CurrentLimits.StatorCurrentLimit = num;
        rollerTalon.getConfigurator().apply(config);
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

    public boolean atGoal(double degTol) {
        return Math.abs(pivot.getError()) < Units.degreesToRadians(degTol);
    }

    public boolean atHandoff() {
        return atGoal(5.0) && this.setpoint.equals(IntakeConstants.HANDOFF_POS)
                && pivot.getSetPoint() == IntakeConstants.HANDOFF_POS.getRadians();
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

            hardSetRoller(rollerSetpoint / IntakeConstants.MAX_KRAKEN_ROLLER_SPEED_M_S); // rollerSetpoint /
                                                                                         // IntakeConstants.MAX_KRAKEN_ROLLER_SPEED_M_S
        } else {
            hardSetPivot(0.75);
            hardSetRoller(0.0);
        }

        // CommandXboxController tester = new CommandXboxController(2);
        // hardSetPivot(tester.getLeftX());
        log();
    }

    public Command getIntakeCommand() {
        return new IntakeCommand();
    }

    public Command getIntakeAmpingCommand() {
        return new IntakeAmp();
    }

    public Command getHomingCommand() {
        return new HomeIntake();
    }

    public Command getIntakeEjaculation() {
        return new TempIntakeEjaculation();
    }

    public Command getIntakeHandoff() {
        return new IntakeHandoff();
    }

    public void log() {
        SteelTalonsLogger.post("Intake pivot angle", pivot.getPosition());
        SteelTalonsLogger.post("Intake pivot setpoint", setpoint.getRadians());
        // SteelTalonsLogger.post("Intake pivot error", pivot.getError());

        // SteelTalonsLogger.post("Intake roller angle", roller.getVelocity());
        // SteelTalonsLogger.post("Intake roller setpoint", this.rollerSetpoint);
        // SteelTalonsLogger.post("Intake roller error", roller.getError());

        // SteelTalonsLogger.post("setpoint velocity", pivot.getSetpointVelocity());

        SteelTalonsLogger.post("Intake Loaded", sensorCovered());
        SteelTalonsLogger.post("intake at goal 5", atGoal(5));
        SteelTalonsLogger.post("intake setpoint roller", rollerSetpoint);
        SteelTalonsLogger.post("Intake Pivot Stalled", pivot.getSmax().getFault(FaultID.kStall));

        // pivot.log();
    }

}