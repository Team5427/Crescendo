package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterHandoff extends Command {

    private Shooter shooter;

    public ShooterHandoff() {
        shooter = Shooter.getInstance();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_HANDOFF);
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_INTAKE_SPEED);
        shooter.setFlywheelSetpoint(0.0, 0.0);

        if (shooter.loaded()) {
            shooter.setFeederSetpoint(ShooterConstants.FEEDER_HOLD_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return shooter.loaded();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_HOLD_SPEED);
        shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_STOW);
        // shooter.setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM, ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
        System.err.println("finished shooter handoff");
    }
}
