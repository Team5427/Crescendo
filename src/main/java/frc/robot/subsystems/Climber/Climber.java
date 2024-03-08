package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Climber extends SubsystemBase {

    private TalonFX rightClimber;
    private TalonFX leftClimber;

    private double setpoint;

    private CommandXboxController commandController;

    private static Climber instance;

    public Climber() {
        //men
        rightClimber = new TalonFX(ClimberConstants.RIGHT_CLIMBER_ID);
        leftClimber = new TalonFX(ClimberConstants.LEFT_CLIMBER_ID);

        ClimberConstants.configureClimber(rightClimber, false);
        ClimberConstants.configureClimber(leftClimber, false);

        setpoint = ClimberConstants.STOW_POSITION;
        commandController = new CommandXboxController(1);

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

    public void setRaw(double left, double right) {
        leftClimber.set(left);
        rightClimber.set(right);
    };

    public void stopBoth() {
        leftClimber.stopMotor();
        leftClimber.stopMotor();
    }

    @Override
    public void periodic() {
        double left = -commandController.getLeftY();
        double right = -commandController.getRightY();
        if (
            DriverStation.isTeleopEnabled() && 
            (Math.abs(left) > 0.5 || 
            Math.abs(right) > 0.5)) {
            
            setRaw(left * 0.8, right * 0.8);
        }
    }

    public Command getClimbCommand() {
        return new ChainClimb();
    }

}
