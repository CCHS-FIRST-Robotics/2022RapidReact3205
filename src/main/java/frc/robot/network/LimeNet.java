package frc.robot.network;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeNet {

    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry tv;
    NetworkTableEntry led_mode;
    NetworkTableEntry cam_mode;
    NetworkTableEntry pipeline;

    public LimeNet() {

    }

    public void setReadState() {
        this.led_mode.setNumber(2);
        this.cam_mode.setNumber(0);
        this.pipeline.setNumber(0);
    }

    public void init(NetworkTableInstance inst) {
        NetworkTable limelight = inst.getTable("Stereo");
        this.tx = limelight.getEntry("tx");
        this.ty = limelight.getEntry("ty");
        this.tv = limelight.getEntry("tv");
        this.led_mode = limelight.getEntry("ledMode");
        this.cam_mode = limelight.getEntry("camMode");
        this.pipeline = limelight.getEntry("pipeline");
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
        double[] ang_arr = { tx.getDouble(0), ty.getDouble(0) };
        return ang_arr;
    }
}
