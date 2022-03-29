package frc.robot.map;
import frc.robot.Constants;
public class InitPos {
    public double[] start_pos = Constants.sp;
    public double heading = Constants.sa;

    public InitPos() {
        this.start_pos[0] = 0;
        this.start_pos[1] = 0;
        //this.heading = -1 * Math.PI * 0.25;
        this.heading = 0;
    }
}
