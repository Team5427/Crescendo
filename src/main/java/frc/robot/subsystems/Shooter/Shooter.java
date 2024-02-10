package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SteelTalonsSparkMaxFlywheel;
import frc.robot.util.SteelTalonsSparkMaxServo;

public class Shooter extends SubsystemBase {

  private SteelTalonsSparkMaxFlywheel leftFlywheel;
  private SteelTalonsSparkMaxFlywheel rightFlywheel;
  private SteelTalonsSparkMaxFlywheel feeder;
  private SteelTalonsSparkMaxServo ampMotor;
  private SteelTalonsSparkMaxServo pivotMaster;
  private SteelTalonsSparkMaxServo pivotSlave;
  /** Creates a new Shooter. */
  public Shooter() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}

