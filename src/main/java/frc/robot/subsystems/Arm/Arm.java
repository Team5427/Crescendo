package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SteelTalonsSparkMaxServo;

public class Arm extends SubsystemBase {

    private SteelTalonsSparkMaxServo armMotor;

    private double setpoint;
    private double armFF;
    
    public Arm() {
        ArmConstants.configureArmMotor();
        armMotor = new SteelTalonsSparkMaxServo(ArmConstants.ARM_MASTER);

        armFF = 0.0;
        setpoint = 0.0;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    @Override
    public void periodic() {
        armMotor.setSetpoint(setpoint, armFF);
        armMotor.log();
    }
}
