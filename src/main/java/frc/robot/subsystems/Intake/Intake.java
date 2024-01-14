package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Intake extends SubsystemBase{

    private CANSparkMax upperRoller;
    private CANSparkMax lowerRoller;

    private static Intake m_instance;

    public Intake() {
        upperRoller = new CANSparkMax(IntakeConstants.UPPER_ROLLER_ID, MotorType.kBrushless);
        lowerRoller = new CANSparkMax(IntakeConstants.LOWER_ROLLER_ID, MotorType.kBrushless);

        upperRoller.setInverted(IntakeConstants.UPPER_INVERTED);
        // lowerRoller.setInverted(IntakeConstants.LOWER_INVERTED);
        lowerRoller.follow(upperRoller);

        m_instance = this;
    }

    public static Intake getInstance() {
        return m_instance;
    }

    public void setMotors(double speed) {
        upperRoller.set(speed);
    }

    public void stopMotors() {
        upperRoller.set(0);
    }

    public Command getCommand(CommandXboxController joy) {
        return new IntakeCommand(joy);
    }
    
}
