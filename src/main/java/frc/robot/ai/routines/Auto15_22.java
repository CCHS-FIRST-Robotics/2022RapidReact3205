package frc.robot.ai.routines;

import java.util.ArrayList;

import frc.robot.coords.*;
import frc.robot.helper.*;
import frc.robot.Constants;

public class Auto15_22 {
    public double[] start_point = { 0, 0 };
    public double start_heading;
    public ArrayList<ArrayList<double[]>> vals = new ArrayList<ArrayList<double[]>>();
    public ArrayList<Double> angsl = new ArrayList<Double>();
    public ArrayList<Methods> met_list = new ArrayList<Methods>();

    public Auto15_22() {
        double[] start = Constants.START;
        double[] ball = Constants.BALL;
        double[] board = Constants.BOARD_PROJECT;

        this.start_point[0] = start[0];
        this.start_point[1] = start[1];
        this.start_heading = start[2];
        double b_h = board[2];

        double[] board_pos = {board[0], board[1]};


        double[] ball_pos = { ball[0], ball[1] };
        double[] ball_back = SimpleMat.add(ball_pos, SimpleMat.projectHeading(ball[2], -1.1));
        double[] ball_diff = SimpleMat.subtract(ball_back, start_point);
        ball_back = SimpleMat.add(ball_back, SimpleMat.scaleVec(SimpleMat.unitVec(ball_diff), 0.1));

        double[] ball_fwd = SimpleMat.add(ball_pos, SimpleMat.projectHeading(ball[2], 0.3));

        this.met_list.add(Methods.SHOOTER_FIRE);
        ArrayList<double[]> empty_n1 = new ArrayList<double[]>();
        this.vals.add(empty_n1);
        this.angsl.add(0.0);

        this.met_list.add(Methods.INTAKE_STORE);
        ArrayList<double[]> empty_0 = new ArrayList<double[]>();
        this.vals.add(empty_0);
        this.angsl.add(0.0);

        this.met_list.add(Methods.RAW_TRANS);
        ArrayList<double[]> pickup0_1 = new ArrayList<double[]>();
        pickup0_1.add(new double[] { 0, 0.4 });
        this.vals.add(pickup0_1);
        this.angsl.add(0.7);

        // this.met_list.add(Methods.TRAVEL);
        // ArrayList<double[]> pickup0_5 = new ArrayList<double[]>();
        // pickup0_5.add(ball_back);
        // this.vals.add(pickup0_5);
        // this.angsl.add(ball[2]);

        // this.met_list.add(Methods.TRAVEL);
        // ArrayList<double[]> pickup0_7 = new ArrayList<double[]>();
        // pickup0_7.add(ball_fwd);
        // this.vals.add(pickup0_7);
        // this.angsl.add(ball[2]);

        // this.met_list.add(Methods.WAIT);
        // ArrayList<double[]> pickup_1 = new ArrayList<double[]>();
        // pickup_1.add(new double[] { 0, 0 });
        // this.vals.add(pickup_1);
        // this.angsl.add(0.5);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> SHA = new ArrayList<double[]>();
        SHA.add(board_pos);
        this.vals.add(SHA);
        this.angsl.add(b_h);

        //this.met_list.add(Methods.RAW_TRANS);
        //ArrayList<double[]> backout = new ArrayList<double[]>();
        //backout.add(new double[] { 0, -0.4});
        //this.vals.add(backout);
        //this.angsl.add(1.);

        this.met_list.add(Methods.SHOOTER_FIRE);
        ArrayList<double[]> empty_4 = new ArrayList<double[]>();
        this.vals.add(empty_4);
        this.angsl.add(0.0);
    }
}
