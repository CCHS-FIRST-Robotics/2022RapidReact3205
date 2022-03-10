package frc.robot.ai.routines;

import java.util.ArrayList;

public class TestCurve {
    public double[] start_point = { 0, 0 };
    public double start_heading;
    public ArrayList<ArrayList<double[]>> vals = new ArrayList<ArrayList<double[]>>();
    public ArrayList<Double> angsl = new ArrayList<Double>();
    public ArrayList<Methods> met_list = new ArrayList<Methods>();

    public TestCurve() {
        this.start_point[0] = 0;
        this.start_point[1] = 0;
        this.start_heading = 0;

        this.met_list.add(Methods.CURVE);
        ArrayList<double[]> t0 = new ArrayList<double[]>();
        t0.add(new double[] { 0.88, 0.3 });
        t0.add(new double[] { 1.43, 0.6 });
        t0.add(new double[] { 1.43, 0.9 });
        t0.add(new double[] { 0.88, 1.2 });
        t0.add(new double[] { 0, 1.5 });
        t0.add(new double[] { -0.88, 1.8 });
        t0.add(new double[] { -1.43, 2.1 });
        t0.add(new double[] { -1.43, 2.4 });
        t0.add(new double[] { -0.88, 2.7 });
        t0.add(new double[] { 0.0, 3 });
        this.vals.add(t0);
        this.angsl.add(-1 * Math.PI / 2);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> t7 = new ArrayList<double[]>();
        t7.add(new double[] { 0, 0 });
        this.vals.add(t7);
        this.angsl.add(0.);
    }
}
