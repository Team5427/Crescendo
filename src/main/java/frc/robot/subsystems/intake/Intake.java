package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.lib.motors.ProfiledSparkMax;
import frc.robot.lib.motors.SteelTalonFX;

public class Intake extends SubsystemBase {
    private ProfiledSparkMax pivotMotor;
    private SteelTalonFX intakeMotor;
    private DigitalInput beamBreak;

    private Intake instance;

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

    }
}
