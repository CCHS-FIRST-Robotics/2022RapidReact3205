package frc.robot.state;

import frc.robot.Constants;

public class Storage {
    public Values val = new Values();
    public Variances var = new Variances();

    public Storage() {
    }

    class Values {
        public double s_lidar_dist = 0.5;

        public Values() {

        }
    }

    class Variances {
        public double s_lidar_dist = Constants.LIDAR_LITE_VAR;
    }
}
