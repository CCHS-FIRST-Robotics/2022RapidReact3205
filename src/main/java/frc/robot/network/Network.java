package frc.robot.network;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.state.MainState;

import java.util.Random;

public class Network {
    NetworkTableEntry start_time;
    NetworkTableEntry emit_time;

    NetworkTableEntry x_pos;
    NetworkTableEntry y_pos;
    NetworkTableEntry heading;
    public StereoNet stereo_net;
    public Lidar lidar;

    public Network() {
        this.stereo_net = new StereoNet();
        this.lidar = new Lidar();
    }

    public void init(double init_time) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable sync = inst.getTable("Sync");

        this.start_time = sync.getEntry("start_time");
        this.start_time.setDouble(init_time);

        this.emit_time = sync.getEntry("emit_time");
        this.emit_time.setDouble((double) System.currentTimeMillis() / 1000);

        NetworkTable state = inst.getTable("State");

        this.x_pos = state.getEntry("x_pos");
        this.x_pos.setDouble(0);

        this.y_pos = state.getEntry("y_pos");
        this.y_pos.setDouble(0);

        this.heading = state.getEntry("heading");
        this.heading.setDouble(0);

        this.stereo_net.init(inst);
        this.lidar.init(inst);
    }

    public void writeNTable(MainState state) {
        this.emit_time.setDouble((double) System.currentTimeMillis() / 1000);

        double[] pos = state.getPosVal();
        this.x_pos.setDouble(pos[0]);
        this.y_pos.setDouble(pos[1]);

        this.heading.setDouble(state.getHeadingVal());
    }
}