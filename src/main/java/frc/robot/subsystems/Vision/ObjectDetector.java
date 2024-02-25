package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectDetector extends SubsystemBase {

    private NetworkTable table_m;
    private boolean tv;

    private static final double inRangeConst = 7.0;
    private static final double xProportional = -0.03; //0.1

    public ObjectDetector(String table) {
        this.table_m = NetworkTableInstance.getDefault().getTable(table);
    }

    @Override
    public void periodic() {
        tv = table_m.getEntry("tv").getDouble(0.0) == 1.0;
        SmartDashboard.putBoolean("entry limelight" + table_m.getPath(), tv);
    }

    public boolean targetVisible() {
        return tv;
    }

    public boolean targetVisible(double id) {
        double ret = table_m.getEntry("tid").getDouble(0.0);
        return id == ret;
    }

    public double[] targetInfo() { // X - Y - A - L
        return new double[] {
            table_m.getEntry("tx").getDouble(0),
            table_m.getEntry("ty").getDouble(0),
            table_m.getEntry("ta").getDouble(0),
            table_m.getEntry("tl").getDouble(0),
        };
    }

    public boolean noteInRange() {
        return tv && (targetInfo()[1] < inRangeConst);
    }

    public double noteDriveAdjustment() {

        double x = targetInfo()[0];

        if (noteInRange()) {
            return x * xProportional; //may need to scale with y too FIXME
        } else {
            return 0.0;
        }
    }

    public double speakerDriveAdjustment(double offsetDeg) {
         if (targetVisible()) {
            return (targetInfo()[0] - offsetDeg) * xProportional;
         } else {
            return 0.0;
         }
    }
}
