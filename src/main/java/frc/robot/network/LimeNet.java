package frc.robot.network;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry tv;

    public Limelight() {

    }

    public void init(NetworkTableInstance inst) {
        NetworkTable limelight = inst.getTable("Stereo");
        this.tx = limelight.getEntry("tx");
        this.ty = limelight.getEntry("ty");
        this.tv = limelight.getEntry("tv");
    }

    public boolean getValid() {
        int val = (int) (this.tv.getDouble(0) + 0.01);
        if (val == 0) {
            return false;
        }
        return true;
    }

    public double[] getAngles() {
        double[] ang_arr = { tx.getDouble(0), ty.getDouble(0) };
        return ang_arr;
    }
}
