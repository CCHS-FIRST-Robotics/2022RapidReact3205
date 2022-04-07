package frc.robot.state;

import frc.robot.Constants;

public class Lime {
    public Values val = new Values();
    public Variances var = new Variances();

    public Lime() {
    }

    class Values {

        public double[] angles = { 0, 0 };
        public double valid = 0;

        public Values() {

        }
    }

    class Variances {
        public double angles = 1;
        public double valid = 1;
    }
}
