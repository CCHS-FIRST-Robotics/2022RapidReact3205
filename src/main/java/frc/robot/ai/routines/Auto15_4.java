package frc.robot.ai.routines;

import java.util.ArrayList;

import frc.robot.helper.*;
import frc.robot.Constants;

public class Auto15_4 {
    public double[] start_point = { 0, 0 };
    public double start_heading;
    public ArrayList<ArrayList<double[]>> vals = new ArrayList<ArrayList<double[]>>();
    public ArrayList<Double> angsl = new ArrayList<Double>();
    public ArrayList<Methods> met_list = new ArrayList<Methods>();

    public Auto15_4() {
        double[] start = Constants.START;
        double[] ball = Constants.BALL;

        double[] far_ball = Constants.BALL;//.FAR_MID;
        double[] far_other = Constants.BALL; //.FAR_LEFT;

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

        this.met_list.add(Methods.SHOOTER_DOUBLE);
        ArrayList<double[]> empty_4 = new ArrayList<double[]>();
        this.vals.add(empty_4);
        this.angsl.add(0.0);

        // Move to 2/3 between
        //this.met_list.add(Methods.ROTATE);
        //ArrayList<double[]> rot2far = new ArrayList<double[]>();
        //rot2far.add(new double[] { far_ball[0], far_ball[1] });
        //this.vals.add(rot2far);
        //this.angsl.add(0.0);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> go2far = new ArrayList<double[]>();
        go2far.add(new double[] { far_ball[0] * 0.5, far_ball[1] * 0.5 });
        this.vals.add(go2far);
        this.angsl.add(far_ball[2]);

        this.met_list.add(Methods.INTAKE_STORE);
        ArrayList<double[]> empty_5 = new ArrayList<double[]>();
        this.vals.add(empty_5);
        this.angsl.add(0.0);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> go2far2 = new ArrayList<double[]>();
        go2far2.add(new double[] { far_ball[0], far_ball[1] });
        this.vals.add(go2far2);
        this.angsl.add(far_ball[2]);

        this.met_list.add(Methods.WAIT);
        ArrayList<double[]> pickup_2 = new ArrayList<double[]>();
        pickup_2.add(new double[] { 0, 0 });
        this.vals.add(pickup_2);
        this.angsl.add(0.5);

        this.met_list.add(Methods.INTAKE_ONLY);
        ArrayList<double[]> empty_6 = new ArrayList<double[]>();
        this.vals.add(empty_6);
        this.angsl.add(0.0);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> go2other = new ArrayList<double[]>();
        go2other.add(new double[] { far_other[0] * 0.9, far_other[1] * 0.9 });
        this.vals.add(go2other);
        this.angsl.add(far_other[2]);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> go2other2 = new ArrayList<double[]>();
        go2other2.add(new double[] { far_other[0], far_other[1] });
        this.vals.add(go2other2);
        this.angsl.add(far_other[2]);

        this.met_list.add(Methods.INTAKE_STATIC);
        ArrayList<double[]> pickup_3 = new ArrayList<double[]>();
        pickup_3.add(new double[] { 0, 0 });
        this.vals.add(pickup_3);
        this.angsl.add(0.5);

        this.met_list.add(Methods.TRAVEL);
        ArrayList<double[]> back = new ArrayList<double[]>();
        back.add(new double[] { far_ball[0] * 0.7, far_ball[1] * 0.7 });
        this.vals.add(back);
        this.angsl.add(far_ball[2]);

        this.met_list.add(Methods.POINT_MID);
        ArrayList<double[]> empty_7 = new ArrayList<double[]>();
        this.vals.add(empty_7);
        this.angsl.add(0.0);

        this.met_list.add(Methods.FIRE_MID);
        ArrayList<double[]> empty_8 = new ArrayList<double[]>();
        this.vals.add(empty_8);
        this.angsl.add(0.0);

        this.met_list.add(Methods.SHOOTER_DOUBLE);
        ArrayList<double[]> empty_9 = new ArrayList<double[]>();
        this.vals.add(empty_9);
        this.angsl.add(0.0);
    }
}
