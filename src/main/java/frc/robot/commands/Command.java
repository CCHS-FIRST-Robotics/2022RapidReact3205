package frc.robot.commands;

public class Command {
    public double fl_pprop = 0;
    public double fr_pprop = 0;
    public double bl_pprop = 0;
    public double br_pprop = 0;
    public double intake_pprop = 0;
    public double storage_1_pprop = 0;

    public Command(double[] values) {
        this.fl_pprop = values[0];
        this.fr_pprop = values[1];
        this.bl_pprop = values[2];
        this.br_pprop = values[3];

        this.intake_pprop = values[4];
        this.storage_1_pprop = values[5];
    }
}
