package frc.robot.ai.subroutines;

import frc.robot.HardwareObjects;
import frc.robot.state.MainState;

public class RIntakeHandler {
    public int substate; // 0 idle, 1 down, 2 up

    public int rin_state; //0 down, 1 up

    double start_time = System.currentTimeMillis()/1000;

    public RIntakeHandler(){
        this.substate = 0;
        this.rin_state = 1;
    }
    public void switchState(){
        if (this.substate != 0){
            this.substate = 0;
            return;
        }
        this.start_time = System.currentTimeMillis()/1000;
        if (this.rin_state == 0){
            this.substate = 1;
        }
        else{
            this.substate = 2;
        }
    }
    public double update(){
        if (this.substate == 0){
            return 0;
        }
        if (this.substate == 1){
            if ((System.currentTimeMillis()/1000) - this.start_time > 0.4){
                this.substate = 0;
            }
            return 1;
        }
        if (this.substate == 2){
            if ((System.currentTimeMillis()/1000) - this.start_time > 0.7){
                this.substate = 0;
            }
            return -1;
        }
        return 0;
    }
}
