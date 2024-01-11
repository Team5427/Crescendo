package frc.robot.subsystems.Misc;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectDetector extends SubsystemBase {
    private NetworkTable table_m;
    private boolean tv;

    private double ledMode;

    public ObjectDetector(NetworkTable table) {
        this.table_m = table;
    }

    @Override
    public void periodic() {
        tv = table_m.getEntry("tv").getDouble(0) == 1;
    }

    public boolean targetVisible() {
        return tv;
    }

    public boolean lightOn() {
        return ledMode == 0;
    }

    public double[] targetInfo() { // X - Y - A - L
        return new double[] {
            table_m.getEntry("tx").getDouble(0),
            table_m.getEntry("ty").getDouble(0),
            table_m.getEntry("ta").getDouble(0),
            table_m.getEntry("tl").getDouble(0),
        };
    }

    public void setLight(boolean on) {
        if (on) {
            table_m.getEntry("ledMode").setNumber(0);
        } else {
            table_m.getEntry("ledMode").setNumber(1);
        }
    }
}
