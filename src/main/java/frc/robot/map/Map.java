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
        hardware.NAVX.setAngleAdjustment(this.pos.heading);

        hardware.IMU.configFactoryDefault();
        hardware.IMU.setFusedHeading(this.pos.heading, Constants.TIMEOUT_MS);

        MainState state = new MainState();
        state.setPos(this.pos.start_pos, state.getPosVar());
        state.setWhlOPos(this.pos.start_pos, state.getWhlOPosVar());

        state.setHeading(this.pos.heading, state.getHeadingVar());
        state.setWhlOHeading(this.pos.heading, state.getWhlOHeadingVar());

        return state;
    }

    public void softInit(MainState state, double[] start_pos, double start_heading) {
        state.setPos(start_pos, state.getPosVar());
        state.setWhlOPos(start_pos, state.getWhlOPosVar());

        state.setHeading(start_heading, state.getHeadingVar());
        state.setWhlOHeading(start_heading, state.getWhlOHeadingVar());

        state.setVel(new double[] { 0, 0 }, Constants.INIT_VARIANCE);
        state.setAcc(new double[] { 0, 0 }, Constants.INIT_VARIANCE);
    }

    public void getBalls(Network net) {
        int live_balls = 0;

        for (int c = 0; c < Constants.BALL_NUM; c++) {
            this.balls[c].ball_pos = net.ball_net.getPosVals(c);
            this.balls[c].ball_vel = net.ball_net.getVelVals(c);
            double[] g_state = net.ball_net.getGStateVals(c);
            this.balls[c].state = (int) (g_state[0] + 0.01);
            this.balls[c].color = (int) (g_state[1] + 0.01);
            this.balls[c].aerial = (int) (g_state[2] + 0.01);
            this.balls[c].fresh = (int) (g_state[3] + 0.01);
            if (this.balls[c].state != 0) {
                live_balls++;
            }
        }
        SmartDashboard.putNumber("Map/Ball Store", live_balls);
    }
}
