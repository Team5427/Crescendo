package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Climber extends SubsystemBase {

    private TalonFX rightClimber;
    private TalonFX leftClimber;

    private double setpoint;

    private static Climber instance;

    public Climber() {
        //men
        rightClimber = new TalonFX(ClimberConstants.RIGHT_CLIMBER_ID);
        leftClimber = new TalonFX(ClimberConstants.LEFT_CLIMBER_ID);

        ClimberConstants.configureClimber(rightClimber);
        ClimberConstants.configureClimber(leftClimber);

        setpoint = ClimberConstants.STOW_POSITION;

        instance = this;
    }

    public static Climber getInstance() {
        return instance;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }
    
    public double getPosition() {
        return ClimberConstants.getClimberRotationsToMeters(leftClimber.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() {

        CommandXboxController test = new CommandXboxController(2);

        double heightChange = test.getHID().getRightTriggerAxis() - test.getHID().getLeftTriggerAxis();
        setpoint += (heightChange > 0.1) ? heightChange: 0;

        PositionVoltage position = new PositionVoltage(setpoint);
        leftClimber.setControl(position);
        rightClimber.setControl(position);
    }

    public Command getClimbCommand() {
        return new ChainClimb();
    }
    
}
