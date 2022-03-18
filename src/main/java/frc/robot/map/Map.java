package frc.robot.map;

import frc.robot.state.MainState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.HardwareObjects;
import frc.robot.network.Network;

public class Map {
    public InitPos pos;
    public double[] hub_pos = { 0, 0 };
    public Obstacle obs;
    public Ball[] balls = new Ball[Constants.BALL_NUM];

    public Map() {
        this.pos = new InitPos();
        this.hub_pos[0] = 0;
        this.hub_pos[1] = 0;
        this.obs = new Obstacle();
        for (int c = 0; c < Constants.BALL_NUM; c++) {
            balls[c] = new Ball();
        }
    }

    public MainState initialize(HardwareObjects hardware) {
        hardware.NAVX.reset();
        hardware.NAVX.zeroYaw();
        hardware.NAVX.setAngleAdjustment(this.pos.heading * 360 / (2 * Math.PI));

        hardware.IMU.configFactoryDefault();
        hardware.IMU.setFusedHeading(this.pos.heading * 360 / (2 * Math.PI), Constants.TIMEOUT_MS);

        MainState state = new MainState();
        state.setPos(this.pos.start_pos, state.getPosVar());
        state.setWhlOPos(this.pos.start_pos, state.getWhlOPosVar());

        state.setHeading(this.pos.heading, state.getHeadingVar());
        state.setWhlOHeading(this.pos.heading, state.getWhlOHeadingVar());

        return state;
    }

    public void softInit(HardwareObjects hardware, MainState state, double[] start_pos, double start_heading) {
        state.setPos(start_pos, state.getPosVar());
        state.setWhlOPos(start_pos, state.getWhlOPosVar());

        state.setHeading(start_heading, state.getHeadingVar());
        state.setWhlOHeading(start_heading, state.getWhlOHeadingVar());

        hardware.IMU.setFusedHeading(start_heading * 360 / (2 * Math.PI));
        hardware.NAVX.setAngleAdjustment(start_heading * 360 / (2 * Math.PI));

        state.setVel(new double[] { 0, 0 }, Constants.INIT_VARIANCE);
        state.setAcc(new double[] { 0, 0 }, Constants.INIT_VARIANCE);
    }

    public void getBalls(Network net) {
        int live_balls = 0;
        int checked = 0;

        for (int c = 0; c < Constants.BALL_NUM; c++) {
            this.balls[c].ball_pos = net.ball_net.getPosVals(c);
            this.balls[c].pos[0] = this.balls[c].ball_pos[0];
            this.balls[c].pos[1] = this.balls[c].ball_pos[1];

            this.balls[c].ball_vel = net.ball_net.getVelVals(c);
            this.balls[c].vel[0] = this.balls[c].ball_vel[0];
            this.balls[c].vel[1] = this.balls[c].ball_vel[1];

            double[] g_state = net.ball_net.getGStateVals(c);
            this.balls[c].state = (int) (g_state[0] + 0.1);
            this.balls[c].color = (int) (g_state[1] + 0.1);
            this.balls[c].aerial = (int) (g_state[2] + 0.1);
            this.balls[c].fresh = (int) (g_state[3] + 0.1);
            if (this.balls[c].state != 0) {
                SmartDashboard.putNumber("GetBall/have", 1);
            }
            if (this.balls[c].state != 0) {
                live_balls++;
            }
            checked++;
        }
        SmartDashboard.putNumberArray("Map/ind 0", net.ball_net.getGStateVals(0));
        SmartDashboard.putNumber("Map/Ball Store", live_balls);
        SmartDashboard.putNumber("Map/Checked", checked);
    }
}