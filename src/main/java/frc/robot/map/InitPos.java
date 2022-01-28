package frc.robot.map;

public class InitPos {
    public double[] start_pos = { 1, 1 };
    public double heading = -1 * Math.PI * 0.25;

    public InitPos() {
        this.start_pos[0] = 0;
        this.start_pos[1] = 0;
        //this.heading = -1 * Math.PI * 0.25;
        this.heading = 0;
    }
}
