package frc.robot.ai.routines;

import java.util.ArrayList;

import frc.robot.ai.routines.coords.*;

import frc.robot.helper.*;

public class Idle {
    public double[] start_point = { 0, 0 };
    public double start_heading;
    public ArrayList<ArrayList<double[]>> vals = new ArrayList<ArrayList<double[]>>();
    public ArrayList<Double> angsl = new ArrayList<Double>();
    public ArrayList<Methods> met_list = new ArrayList<Methods>();

    public Idle() {
        double[] start = SB3_B3.START;
        double[] ball = SB3_B3.BALL;

        double[] ball_pos = {ball[0], ball[1]};
        double[] ball_back = SimpleMat.add(ball_pos, SimpleMat.projectHeading(ball[2], -0.5));
        double[] ball_fwd = SimpleMat.add(ball_pos, SimpleMat.projectHeading(ball[2], 0.3));

        this.start_point[0] = start[0];
        this.start_point[1] = start[1];
        this.start_heading = start[2];


        this.met_list.add(Methods.SHOOTER_FIRE);
        ArrayList<double[]> empty_4 = new ArrayList<double[]>();
        this.vals.add(empty_4);
        this.angsl.add(0.0);

        this.met_list.add(Methods.DROP_INTAKE);
        ArrayList<double[]> pickup0_1 = new ArrayList<double[]>();
        pickup0_1.add(new double[] { 0, 0 });
        this.vals.add(pickup0_1);
        this.angsl.add(0.0);

        this.met_list.add(Methods.WAIT);
        ArrayList<double[]> pickup_1 = new ArrayList<double[]>();
        pickup_1.add(new double[] { 0, 0 });
        this.vals.add(pickup_1);
        this.angsl.add(1.);

        this.met_list.add(Methods.RAW_TRANS);
        ArrayList<double[]> backout = new ArrayList<double[]>();
        backout.add(new double[] { 0, 0.4});
        this.vals.add(backout);
        this.angsl.add(1.);
    }
}
