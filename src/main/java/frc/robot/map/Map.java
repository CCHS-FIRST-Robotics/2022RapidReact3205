package frc.robot.map;

import frc.robot.state.MainState;
import frc.robot.HardwareObjects;

public class Map {
    public InitPos pos;
    public double[] hub_pos = { 0, 0 };
    public Obstacle obs;

    public Map() {
        this.pos = new InitPos();
        this.hub_pos[0] = 0;
        this.hub_pos[1] = 0;
        this.obs = new Obstacle();
    }

    public MainState initialize(HardwareObjects hardware) {
        hardware.NAVX.reset();
        hardware.NAVX.calibrate();
        while (hardware.NAVX.isCalibrating()) {

        }
        hardware.NAVX.zeroYaw();
        hardware.NAVX.setAngleAdjustment(this.pos.heading);

        MainState state = new MainState();
        state.setPos(this.pos.start_pos, state.getPosVar());
        state.setWhlOPos(this.pos.start_pos, state.getWhlOPosVar());

        state.setHeading(this.pos.heading, state.getHeadingVar());
        state.setWhlOHeading(this.pos.heading, state.getWhlOHeadingVar());

        return state;
    }
}
