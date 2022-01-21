package frc.robot.commands;

public class Command {
    public double fl_pprop = 0;
    public double fr_pprop = 0;
    public double bl_pprop = 0;
    public double br_pprop = 0;

    public Command(double fl, double fr, double bl, double br) {
        this.fl_pprop = fl;
        this.fr_pprop = fr;
        this.bl_pprop = bl;
        this.br_pprop = br;
    }
}
