package frc.robot.ai.routines;

import java.util.ArrayList;

import frc.robot.ai.routines.coords.*;

public class Auto15_2 {
    public double[] start_point = { 0, 0 };
    public double start_heading;
    public ArrayList<ArrayList<double[]>> vals = new ArrayList<ArrayList<double[]>>();
    public ArrayList<Double> angsl = new ArrayList<Double>();
    public ArrayList<Methods> met_list = new ArrayList<Methods>();

    public Auto15_2() {
        double[] start = SB1_B1.START;
        double[] ball = SB1_B1.BALL;

        this.start_point[0] = start[0];
        this.start_point[1] = start[1];
        this.start_heading = start[2];

        this.met_list.add(Methods.INTAKE_ONLY);
        ArrayList<double[]> empty_0 = new ArrayList<double[]>();
        this.vals.add(empty_0);
        this.angsl.add(0.0);

        this.met_list.add(Methods.DROP_INTAKE);
        ArrayList<double[]> pickup0_1 = new ArrayList<double[]>();
        pickup0_1.add(new double[] { 0, 0 });
        this.vals.add(pickup0_1);
        this.angsl.add(0.0);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> pickup0_5 = new ArrayList<double[]>();
        pickup0_5.add(new double[] { ball[0], ball[1] });
        this.vals.add(pickup0_5);
        this.angsl.add(ball[2]);

        this.met_list.add(Methods.WAIT);
        ArrayList<double[]> pickup_1 = new ArrayList<double[]>();
        pickup_1.add(new double[] { 0, 0 });
        this.vals.add(pickup_1);
        this.angsl.add(0.5);

        this.met_list.add(Methods.POINT_MID);
        ArrayList<double[]> empty_2 = new ArrayList<double[]>();
        this.vals.add(empty_2);
        this.angsl.add(0.0);

        this.met_list.add(Methods.FIRE_MID);
        ArrayList<double[]> empty_3 = new ArrayList<double[]>();
        this.vals.add(empty_3);
        this.angsl.add(0.0);

        this.met_list.add(Methods.FIRE_MID);
        ArrayList<double[]> empty_3_5 = new ArrayList<double[]>();
        this.vals.add(empty_3_5);
        this.angsl.add(0.0);

        this.met_list.add(Methods.SHOOTER_DOUBLE);
        ArrayList<double[]> empty_4 = new ArrayList<double[]>();
        this.vals.add(empty_4);
        this.angsl.add(0.0);
    }
}
