package frc.robot.subsystems.managing;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShootingConfiguration;
import frc.robot.util.MiscUtil;

public class AutonShoot extends Command {
    private Shooter shooter;
    private Timer timer;

    public AutonShoot() {
        shooter = Shooter.getInstance();
        timer = new Timer();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        ShootingConfiguration config = ShooterConstants.SHOOTER_PIVOT_TARGET_MAP.get(MiscUtil.targetingInformation()[2]);
        shooter.setShootingConfigSetpoints(config);

        if (timer.get() > 0.5 && shooter.flywheelAtGoal()) {
            shooter.setFeederSetpoint(ShooterConstants.FEEDER_FEED_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        return !shooter.loaded() && timer.get() > 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_HANDOFF);
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_HOLD_SPEED);
    }
}
