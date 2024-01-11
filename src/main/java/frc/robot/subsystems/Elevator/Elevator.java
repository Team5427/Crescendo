package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SteelTalonsSparkMaxServo;

public class Elevator extends SubsystemBase {

    private SteelTalonsSparkMaxServo elevatorMotor;

    private double setpoint;
    private double elevatorFF;

    public Elevator() {
        ElevatorConstants.configureElevatorMotor();
        elevatorMotor = new SteelTalonsSparkMaxServo(ElevatorConstants.ELEVATOR_MASTER);

        elevatorFF = 0.0;
        setpoint = 0.0;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    @Override
    public void periodic() {
        elevatorMotor.setSetpoint(setpoint, elevatorFF);
        elevatorMotor.log();
    }
    
}
