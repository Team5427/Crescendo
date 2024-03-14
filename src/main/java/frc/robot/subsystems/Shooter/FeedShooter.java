package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class FeedShooter extends Command {

    private Shooter shooter;
    private double setpoint = 0.0;
    private Rotation2d pivotRot;
    private boolean startedFeeder = false;
    private boolean useNums = false;

    public FeedShooter(double setpoint, Rotation2d pivotRot, boolean useNums) {
        shooter = Shooter.getInstance();

        this.setpoint = setpoint;
        this.pivotRot = pivotRot;

        this.useNums = useNums;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        if (useNums) {
            shooter.setPivotSetpoint(pivotRot);
        }
    }

    @Override
    public void execute() {
        if (useNums) {
            shooter.setFlywheelSetpoint(setpoint, setpoint);
        }
        if (shooter.pivotAtGoal(1.0) && shooter.flywheelAtGoal()) {
            shooter.setFeederSetpoint(ShooterConstants.FEEDER_FEED_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return 
        !shooter.loaded();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_HOLD_SPEED);
        shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_STOW);
        shooter.setFlywheelSetpoint(ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM, ShooterConstants.FLYWHEEL_STATIC_SPEED_RPM);
    }
}
