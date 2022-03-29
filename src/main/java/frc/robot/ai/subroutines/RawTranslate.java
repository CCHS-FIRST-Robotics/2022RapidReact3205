package frc.robot.ai.subroutines;

import frc.robot.helper.*;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.state.MainState;
public class RawTranslate {
    double[] vel_vec;
    double total_time;
    double start_time;
    DPID fl;
    DPID fr;
    DPID bl;
    DPID br;
    public RawTranslate(double[] dir_vec, double time){
        //cardinal is nsew
        double v_max = Constants.WHEEL_RADIUS * Constants.MOTOR_MAX_RPM * 2 * Math.PI
        / 60;
        this.vel_vec = SimpleMat.scaleVec(dir_vec, v_max);
        this.start_time = System.currentTimeMillis()/1000;
        this.total_time = time;
        this.fl = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.fr = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.bl = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
        this.br = new DPID(Constants.C_BASE_PID[0], Constants.C_BASE_PID[1], Constants.C_BASE_PID[2]);
    }
    public Command update(MainState state){
        double[] whl_array = MecanumIK.mecanumIK(this.vel_vec, 0);
        double flr = this.fl.updateRaw(whl_array[0], state.getFLRadssVal());
        double frr = this.fr.updateRaw(whl_array[1], state.getFRRadssVal());
        double blr = this.bl.updateRaw(whl_array[2], state.getBLRadssVal());
        double brr = this.br.updateRaw(whl_array[3], state.getBRRadssVal());
        double[] ocmd = { flr, frr, blr, brr, 0, 0, 0, 0, 0, 0, 0 };
        return new Command(ocmd);
    }
    public boolean exit(){
        if (System.currentTimeMillis()/1000 - this.start_time > this.total_time){
            return true;
        }
        return false;
    }
}
