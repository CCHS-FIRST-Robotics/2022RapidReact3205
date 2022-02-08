package frc.robot.map;

import frc.robot.helper.*;
import java.util.ArrayList;

public class Obstacle {
    // 0.5x0.5 cells 17 long 33 wide
    // cell at 0,0
    // 0 for clear, 1 for immutable (walls/ tower etc), 2 for commutable (robots
    // etc), 3 for balls
    int[][] obstacle_cells = new int[17][33];
    ArrayList<int[]> mutable = new ArrayList<int[]>();

    public Obstacle() {
        for (int iy = 0; iy < 17; iy++) {
            for (int ix = 0; ix < 33; ix++) {
                // border
                if (iy == 0 || iy == 16 || ix == 0 || ix == 32) {
                    this.obstacle_cells[iy][ix] = 1;
                }
                double[] o_vec = { (ix - 9) * 0.5, (iy - 17) * 0.5 };
                if (SimpleMat.mag(o_vec) < 2) {
                    this.obstacle_cells[iy][ix] = 1;
                }
            }
        }
    }

    public int[] pos2cell(double[] pos) {
        int[] cell = { (int) Math.round(pos[0] * 2) + 9, (int) Math.round(pos[1] * 2) + 17 };
        return cell;
    }

    public void addMutable(int[] obst_cpa) {
        // cpa is 3 elements, (x,y,state), state = 0 means ignore, state = 1 means
        // remove
        for (int c = 0; c < obst_cpa.length; c++) {
            if (obst_cpa[2] == 1) {
                int existing = this.obstacle_cells[obst_cpa[1]][obst_cpa[0]];
                if (existing == 0) {
                    this.obstacle_cells[obst_cpa[1]][obst_cpa[0]] = 2;
                    int[] coords = { obst_cpa[0], obst_cpa[1] };
                    this.mutable.add(coords);
                }
            }
        }
    }

    public void removeMutable() {
        for (int c = this.mutable.size() - 1; c > -1; c--) {
            int[] coords = this.mutable.get(c);
            this.obstacle_cells[coords[1]][coords[0]] = 0;
            this.mutable.remove(c);
        }
    }
}
