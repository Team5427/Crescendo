package frc.robot.subsystems.managing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class Unstuck extends Command {
    Shooter shooter;
    Intake intake;

    public Unstuck() {
        shooter = Shooter.getInstance();
        intake = Intake.getInstance();
        addRequirements(shooter, intake);
    }

    @Override
    public void initialize() {
        shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_HANDOFF);
        intake.setPivotSetpoint(IntakeConstants.HANDOFF_POS);
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_EJECTING);
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_INTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        return shooter.inPosition();
    } 
    
    @Override
    public void end(boolean interrupted) {
        shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_STOW);
        intake.setPivotSetpoint(IntakeConstants.STOWED_POS);
        intake.setRollerSetpoint(IntakeConstants.INTAKE_SPEED_HOLD);
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_HOLD_SPEED);
    }
}
