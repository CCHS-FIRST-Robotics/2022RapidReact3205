package frc.robot.ai.routines;

import java.util.ArrayList;

public class SR1_R1 {
    public double[] start_point = { 0, 0 };
    public double start_heading;
    public ArrayList<ArrayList<double[]>> vals = new ArrayList<ArrayList<double[]>>();
    public ArrayList<Double> angsl = new ArrayList<Double>();
    public ArrayList<Methods> met_list = new ArrayList<Methods>();

    public SR1_R1() {
        this.start_point[0] = 0.661;
        this.start_point[1] = 3;
        this.start_heading = 0;

        this.met_list.add(Methods.INTAKE_ONLY);
        ArrayList<double[]> empty_0 = new ArrayList<double[]>();
        this.vals.add(empty_0);
        this.angsl.add(0.0);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> pickup0 = new ArrayList<double[]>();
        pickup0.add(new double[] { 0.661, 3.9 });
        this.vals.add(pickup0);
        this.angsl.add(0.0);

        this.met_list.add(Methods.INTAKE_IDLE);
        ArrayList<double[]> empty_1 = new ArrayList<double[]>();
        this.vals.add(empty_1);
        this.angsl.add(0.0);

        this.met_list.add(Methods.POINT_MID);
        ArrayList<double[]> empty_2 = new ArrayList<double[]>();
        this.vals.add(empty_2);
        this.angsl.add(0.0);

        this.met_list.add(Methods.FIRE_MID);
        ArrayList<double[]> empty_3 = new ArrayList<double[]>();
        this.vals.add(empty_3);
        this.angsl.add(0.0);

        this.met_list.add(Methods.SHOOTER_DOUBLE);
        ArrayList<double[]> empty_4 = new ArrayList<double[]>();
        this.vals.add(empty_4);
        this.angsl.add(0.0);

    }
}