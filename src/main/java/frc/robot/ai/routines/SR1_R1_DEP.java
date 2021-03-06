package frc.robot.ai.routines;

import java.util.ArrayList;

public class SR1_R1_DEP {
    public double[] start_point = { 0, 0 };
    public double start_heading;
    public ArrayList<ArrayList<double[]>> vals = new ArrayList<ArrayList<double[]>>();
    public ArrayList<Double> angsl = new ArrayList<Double>();
    public ArrayList<Methods> met_list = new ArrayList<Methods>();

    public SR1_R1_DEP() {
        this.start_point[0] = 1.243;
        this.start_point[1] = 1.94;
        this.start_heading = 0;

        this.met_list.add(Methods.INTAKE_ONLY);
        ArrayList<double[]> empty_0 = new ArrayList<double[]>();
        this.vals.add(empty_0);
        this.angsl.add(0.0);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> pickup0_1 = new ArrayList<double[]>();
        pickup0_1.add(new double[] { 1.68, 1.5 });
        this.vals.add(pickup0_1);
        this.angsl.add(0.0);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> pickup0 = new ArrayList<double[]>();
        pickup0.add(new double[] { 1.68, 2.5 });
        this.vals.add(pickup0);
        this.angsl.add(0.0);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> pickup0_5 = new ArrayList<double[]>();
        pickup0_5.add(new double[] { 1.661, 3.4 });
        this.vals.add(pickup0_5);
        this.angsl.add(0.0);

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