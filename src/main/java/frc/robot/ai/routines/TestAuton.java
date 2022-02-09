package frc.robot.ai.routines;

import java.util.ArrayList;

public class TestAuton {
    public static ArrayList<ArrayList<double[]>> sinCurveVals() {
        ArrayList<double[]> curve = new ArrayList<double[]>();
        curve.add(new double[] { 0.88, 0.3 });
        curve.add(new double[] { 1.43, 0.6 });
        curve.add(new double[] { 1.43, 0.9 });
        curve.add(new double[] { 0.88, 1.2 });
        curve.add(new double[] { 0, 1.5 });
        curve.add(new double[] { -0.88, 1.8 });
        curve.add(new double[] { -1.43, 2.1 });
        curve.add(new double[] { -1.43, 2.4 });
        curve.add(new double[] { -0.88, 2.7 });
        curve.add(new double[] { 0.0, 3 });
        ArrayList<ArrayList<double[]>> vals = new ArrayList<ArrayList<double[]>>();
        vals.add(curve);
        return vals;
    }

    public static ArrayList<Double> sinCurveAngs() {
        ArrayList<Double> angsl = new ArrayList<Double>();
        angsl.add(0.0);
        return angsl;
    }

    public static ArrayList<Methods> sinCurveMethods() {
        ArrayList<Methods> met_list = new ArrayList<Methods>();
        met_list.add(Methods.CURVE);
        return met_list;
    }
}
