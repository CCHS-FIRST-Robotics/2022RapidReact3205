package frc.robot.network;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class Ball {
    NetworkTableEntry[] pos = new NetworkTableEntry[Constants.BALL_NUM];
    NetworkTableEntry[] vel = new NetworkTableEntry[Constants.BALL_NUM];
    NetworkTableEntry[] grand_state = new NetworkTableEntry[Constants.BALL_NUM];

    public Ball() {

    }

    public void init(NetworkTableInstance inst) {
        NetworkTable balls = inst.getTable("Balls");
        for (int c = 0; c < Constants.BALL_NUM; c++) {
            String pos_str = "pos_" + Integer.toString(c);
            String vel_str = "vel_" + Integer.toString(c);
            String gstate_str = "gstate_" + Integer.toString(c);
            this.pos[c] = balls.getEntry(pos_str);
            this.vel[c] = balls.getEntry(vel_str);
            this.grand_state[c] = balls.getEntry(gstate_str);
        }
    }

    public double[] getPosVals(int ent) {
        return this.pos[ent].getDoubleArray(new double[] { 0., 0., 0. });
    }

    public double[] getVelVals(int ent) {
        return this.vel[ent].getDoubleArray(new double[] { 0., 0., 0. });
    }

    public double[] getGStateVals(int ent) {
        return this.grand_state[ent].getDoubleArray(new double[] { 0, 0, 0 });
    }
}
