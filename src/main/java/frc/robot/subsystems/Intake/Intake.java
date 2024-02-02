package frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.SteelTalonsLogger;
import frc.robot.util.SteelTalonsSparkMaxFlywheel;
import frc.robot.util.SteelTalonsSparkMaxServo;

public class Intake extends SubsystemBase {

    private SteelTalonsSparkMaxFlywheel roller;
    private SteelTalonsSparkMaxServo pivot;
    private DigitalInput beamBreaker;

    private double rollerSetpoint = 0.0;
    private Rotation2d setpoint = new Rotation2d();

    private static Intake instance;

    private boolean isHoming;

    public Intake () {
        IntakeConstants.configureIntake();
        roller = new SteelTalonsSparkMaxFlywheel(IntakeConstants.ROLLER_CONFIG);
        pivot = new SteelTalonsSparkMaxServo(IntakeConstants.PIVOT_CONFIG);
        pivot.disableContinuousInput();
        resetPivotEncoder(IntakeConstants.HARDSTOP_POS);
        instance = this;

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

    public SteelTalonsSparkMaxFlywheel getRoller() {
        return roller;
    }

    public SteelTalonsSparkMaxServo getPivot() {
        return pivot;
    }

    public void stopRoller() {
        roller.forceStop();
    }

    public void hardSetPivot(double percent) {
        pivot.setRaw(percent);
    }

    public void hardSetRoller(double percent) {
        roller.setRaw(percent);
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
            );

            roller.setSetpoint(rollerSetpoint, 0.0);
        }

        CommandXboxController controller = new CommandXboxController(1);
        if (controller.getHID().getRightBumper()) {
            hardSetRoller(controller.getRightTriggerAxis() > 0.1 ? controller.getRightTriggerAxis() : 0.0);
        }
        // log();
    }

    public Command getBasicIntakeCommand(CommandXboxController controller) {
        return new IntakeCommand(controller);
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