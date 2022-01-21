package frc.robot.ai.subroutines.exit_methods;

import frc.robot.Constants;
import frc.robot.helper.SimpleMat;
import frc.robot.state.MainState;

public class ExitMethods {
    public static double timeSinceStart(double start_time) {
        long current_time_sec = System.currentTimeMillis() / 1000;
        double difference = current_time_sec - start_time;
        double timeDifClean = Double.parseDouble(String.format("%.2g%n", difference)); // Time in seconds to hundreths
        return timeDifClean;
    }

    public static double distanceToEnd(MainState main_state, double[] final_pos) {
        double[] pos = main_state.getPosVal();
        double distance = Math.sqrt(Math.pow((final_pos[0] - pos[0]), 2) + Math.pow((final_pos[1] - pos[1]), 2));
        double disClean = Double.parseDouble(String.format("%.2g%n", distance)); // Rounded to hundreths
        return disClean;
    }

    public static double targetTime(double travel_distance) {
        return Constants.TIMER_START + travel_distance * Constants.TIMER_LEEWAY
                / ((Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60) * Constants.WHEEL_RADIUS);
    }

    public static double initScoreCoeff(double max_time) {
        return max_time * Constants.ACCEPTABLE_DIST_ERROR;
    }

    public static boolean fusionExit(double vel_mag, double t_dist) {
        double min_vel_mag = 0.1 * ((Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60) * Constants.WHEEL_RADIUS);
        double a_vel = Math.max(vel_mag, min_vel_mag);
        double n_coeff = Constants.ACCEPTABLE_DIST_ERROR * Constants.ACCEPTABLE_DIST_ERROR / a_vel;
        if (t_dist / a_vel < n_coeff / t_dist) {
            return true;
        }
        return false;
    }

    public static boolean thetaExit(MainState state, double[] target) {
        double[] delta = { target[0] - state.getPosVal()[0], target[1] - state.getPosVal()[1] };
        double[] heading = SimpleMat.projectHeading(state.getHeadingVal(), 1);
        double smaller_angle = Math.acos(SimpleMat.dot(delta, heading) / SimpleMat.mag(delta));
        return (smaller_angle * 2 > Constants.EXIT_THETA);
    }

    public static boolean timeExit(MainState main_state) {
        // if (timeSinceStart() * [rps * wheelcircumfrence] == traveldistance) { return
        // true }
        return false;
    }

    public static boolean distanceExit(MainState main_state) {
        // if (distanceToEnd(main_state) == 0) {
        // return true;
        // }
        return false;
    }

    public static boolean weightedEnd() { // VERY ROUGH
        // if ((timeSinceStart() * [rps * wheelcircumfrence] / traveldistance) * 10 +
        // ((this.traveldistance - distanceToEnd) / (this.traveldistance / 10)) * 10 >=
        // 20 ) {return true}
        return false;
    }
}
