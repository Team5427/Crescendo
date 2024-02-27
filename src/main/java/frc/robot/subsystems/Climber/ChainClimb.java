package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class ChainClimb extends Command {

    private Climber climber;
    private boolean forceQuit;

    public ChainClimb() {
        climber = Climber.getInstance();
        addRequirements(climber);

        forceQuit = climber.getPosition() < ClimberConstants.CLIMB_POSITION - ClimberConstants.CLIMB_TOLERANCE;
    }

    @Override
    public void execute() {
        climber.setSetpoint(ClimberConstants.STOW_POSITION);
    }

    @Override
    public boolean isFinished() {
        return forceQuit || climber.getPosition() < ClimberConstants.STOW_POSITION + ClimberConstants.CLIMB_TOLERANCE;
    }
    
}
