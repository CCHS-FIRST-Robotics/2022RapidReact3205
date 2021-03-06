package frc.robot.network;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeNet {

    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry tv;
    NetworkTableEntry thor;
    NetworkTableEntry tver;
    NetworkTableEntry led_mode;
    NetworkTableEntry cam_mode;
    NetworkTableEntry pipeline;

    public LimeNet() {

    }

    public void setReadState() {
    }

    public void init(NetworkTableInstance inst) {
        NetworkTable limelight = inst.getTable("limelight");
        this.tx = limelight.getEntry("tx");
        this.ty = limelight.getEntry("ty");
        this.tv = limelight.getEntry("tv");
        this.thor = limelight.getEntry("thor");
        this.tver = limelight.getEntry("tver");
    }

    public boolean getValid() {
        setReadState();
        int val = (int) (this.tv.getDouble(0) + 0.01);
        if (val == 0) {
            return false;
        }
        return true;
    }

    public double[] getAngles() {
        setReadState();
        double[] ang_arr = { tx.getDouble(0) + 8, ty.getDouble(0) };
        return ang_arr;
    }

    public double[] getRect() {
        setReadState();
        double[] rect_arr = { thor.getDouble(0), tver.getDouble(0) };
        return rect_arr;
    }
}
