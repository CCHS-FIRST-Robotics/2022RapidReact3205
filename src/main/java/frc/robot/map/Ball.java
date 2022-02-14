package frc.robot.map;

import frc.robot.network.Network;

public class Ball {
    public double[] ball_pos = { 0, 0, 0 };
    double[] ball_vel = { 0, 0, 0 };
    public int state = 0;
    public int aerial = 0;
    public int color = 0;

    public Ball() {
        this.ball_pos[0] = 0;
        this.ball_pos[1] = 0;
        this.ball_pos[2] = 0;

        this.ball_vel[0] = 0;
        this.ball_vel[1] = 0;
        this.ball_vel[2] = 0;

        this.state = 0;
        this.aerial = 0;
        this.color = 0;
    }

    public void getVals(Network net) {

    }
}
