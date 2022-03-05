package frc.robot.ai.subroutines;

import frc.robot.state.MainState;

public class IntakeHandler {
    // substates: 0, idle, 1: picking up into storage, 2: picking up into 3: storing
    public int substate = 0;
    double start_time = System.currentTimeMillis() / 1000;
    double storage_time = System.currentTimeMillis() / 1000;

    public IntakeHandler() {
        this.substate = 0;
    }

    public void idle() {
        this.substate = 0;
    }

    public void intakeOnly() {
        this.substate = 1;
        this.start_time = System.currentTimeMillis() / 1000;
    }

    public void intakeStorage() {
        this.substate = 2;
        this.start_time = System.currentTimeMillis() / 1000;
    }

    public double[] update(MainState state) {
        double c_time = System.currentTimeMillis() / 1000;
        if (this.substate == 0) {
            return new double[] { 0, 0, 0.1 };
        }
        if (this.substate == 1) {
            if (state.getSLIDARVal() < 0.3) {
                this.substate = 0;
            }
            return new double[] { 1, 0.3, 0.1 };
        }
        if (this.substate == 2) {
            if ( state.getBeam1Val() == 1) {
                this.substate = 3;
                this.storage_time = System.currentTimeMillis() / 1000;
            }
            return new double[] { 0.5, 0.3, 0.2 };
        }
        if (this.substate == 3) {
            if (c_time - this.storage_time > 0.00) {
                this.substate = 0;
            }
            return new double[] { 0.0, 0.0, 0.0 };
        }
        return new double[] { 0, 0, 0.1 };
    }

    public boolean exit() {
        if (this.substate == 2 || this.substate == 3) {
            return false;
        }
        return true;
    }
}
