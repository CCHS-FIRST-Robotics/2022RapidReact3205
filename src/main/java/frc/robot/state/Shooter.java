package frc.robot.state;

import frc.robot.Constants;

public class Shooter {
    public Values val = new Values();
    public Variances var = new Variances();

    public Shooter() {
    }

    class Values {
        public double[] shooter_radss = { 0, 0 };
        public double storage_radss = 0;

        public Values() {

        }
    }

    class Variances {
        public double shooter_radss = Constants.INIT_VARIANCE;
        public double storage_radss = Constants.INIT_VARIANCE;
    }
}
