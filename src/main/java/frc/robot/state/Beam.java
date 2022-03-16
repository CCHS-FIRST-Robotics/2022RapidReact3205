package frc.robot.state;

import frc.robot.Constants;

public class Beam {
    public Values val = new Values();
    public Variances var = new Variances();

    public Beam() {
    }

    class Values {

        public double beam_0 = 0;

        public double beam_1 = 0;

        public double beam_0_5 = 0;

        public Values() {

        }
    }

    class Variances {
        public double beam_0 = 0;
        public double beam_1 = 1;
        public double beam_0_5 = 1;
    }
}
