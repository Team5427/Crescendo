package frc.robot.subsystems.managing;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShootingConfiguration;

public class AutonShoot extends Command {
    private Shooter shooter;
    private Timer timer;
    private boolean useVision;
    private PIDController visionPID;

    public AutonShoot(boolean useVision) {
        shooter = Shooter.getInstance();
        timer = new Timer();
        addRequirements(shooter);
        this.useVision = useVision;
        visionPID = new PIDController(1.5, 0, 0);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

    }

    @Override
    public void execute() {
        ShootingConfiguration config = ShooterConstants.SHOOTER_PIVOT_TARGET_MAP.get(RobotContainer.getTagCam().speakerDist());

        shooter.setShootingConfigSetpoints(config); //ONLY FOR TESTING RN

        if (timer.get() > 0.25 && shooter.flywheelAtGoal() && shooter.pivotAtGoal(1.0)) {
            shooter.setFeederSetpoint(ShooterConstants.FEEDER_FEED_SPEED);
        }

        // if (useVision) {
        //     frc.robot.subsystems.Swerve.SwerveDrivetrain
        // }
    }

    @Override
    public boolean isFinished() {
        return !shooter.loaded() && timer.get() > 0.5;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setPivotSetpoint(ShooterConstants.SHOOTER_PIVOT_HANDOFF);
        shooter.setFeederSetpoint(ShooterConstants.FEEDER_HOLD_SPEED);
    }
}
