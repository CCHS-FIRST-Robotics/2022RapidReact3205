package frc.robot.ai.routines;

import java.util.ArrayList;

public class TestTravel {
    public double[] start_point = { 0, 0 };
    public double start_heading;
    public ArrayList<ArrayList<double[]>> vals = new ArrayList<ArrayList<double[]>>();
    public ArrayList<Double> angsl = new ArrayList<Double>();
    public ArrayList<Methods> met_list = new ArrayList<Methods>();

    public TestTravel() {
        this.start_point[0] = 0;
        this.start_point[1] = 0;
        this.start_heading = 0;

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> t0 = new ArrayList<double[]>();
        t0.add(new double[] { 1.5, 1.5 });
        this.vals.add(t0);
        this.angsl.add(-1 * Math.PI / 2);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> t1 = new ArrayList<double[]>();
        t1.add(new double[] { 1, 1 });
        this.vals.add(t1);
        this.angsl.add(-1.5 * Math.PI / 2);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> t2 = new ArrayList<double[]>();
        t2.add(new double[] { 0, 0 });
        this.vals.add(t2);
        this.angsl.add(-2 * Math.PI / 2);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> t3 = new ArrayList<double[]>();
        t3.add(new double[] { -1, 0 });
        this.vals.add(t3);
        this.angsl.add(0.);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> t4 = new ArrayList<double[]>();
        t4.add(new double[] { -1.5, 1.5 });
        this.vals.add(t4);
        this.angsl.add(-1 * Math.PI / 2);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> t5 = new ArrayList<double[]>();
        t5.add(new double[] { -1.5, 0 });
        this.vals.add(t5);
        this.angsl.add(0.);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> t6 = new ArrayList<double[]>();
        t6.add(new double[] { -1.5, -1.5 });
        this.vals.add(t6);
        this.angsl.add(1 * Math.PI / 2);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> t7 = new ArrayList<double[]>();
        t7.add(new double[] { 0, 0 });
        this.vals.add(t7);
        this.angsl.add(0.);
    }
}