package frc.robot.network;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Lidar {
    NetworkTableEntry p_x_pos;
    NetworkTableEntry p_y_pos;
    NetworkTableEntry p_heading;
    NetworkTableEntry pos_var;
    NetworkTableEntry h_var;
    NetworkTableEntry dt;
    NetworkTableEntry updated;

    Lidar() {

    }

    public void init(NetworkTableInstance inst) {
        NetworkTable stereo = inst.getTable("lidar");

        this.p_x_pos = stereo.getEntry("p_x_pos");
        this.p_x_pos.setDouble(0);

        this.p_y_pos = stereo.getEntry("p_y_pos");
        this.p_y_pos.setDouble(0);

        this.p_heading = stereo.getEntry("p_heading");
        this.p_heading.setDouble(0);

        this.pos_var = stereo.getEntry("pos_var");
        this.pos_var.setDouble(1000);

        this.h_var = stereo.getEntry("h_var");
        this.h_var.setDouble(1000);
 
        this.dt = stereo.getEntry("dt");
        this.dt.setDouble(0);

        this.updated = stereo.getEntry("updated");
        this.updated.setBoolean(false);
    }

    public boolean updatedQ() {
        if (this.updated.getBoolean(false)) {
            this.updated.setBoolean(false);
            return true;
        }
        return false;
    }

    public double[] getPosVal() {
        double[] o = { 0, 0 };
        o[0] = this.p_x_pos.getDouble(0);
        o[1] = this.p_y_pos.getDouble(0);
        return o;
    }

    public double getHeadingVal() {
        return this.p_heading.getDouble(0);
    }

    public double getPosVar() {
        return this.pos_var.getDouble(1000);
    }

    public double getHeadingVar() {
        return this.h_var.getDouble(1000);
    }
}