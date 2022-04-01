package frc.robot.commands;

public class Command {
    public double fl_pprop = 0;
    public double fr_pprop = 0;
    public double bl_pprop = 0;
    public double br_pprop = 0;
    public double intake_pprop = 0;
    public double storage_1_pprop = 0;

    public double storage_2_pprop = 0;
    public double shooter1_pprop = 0;
    public double shooter2_pprop = 0;

    public double hang_l_pprop = 0;
    public double hang_r_pprop = 0;

    public double rin_prop = 0;

    public Command(double[] values) {
        this.fl_pprop = values[0];
        this.fr_pprop = values[1];
        this.bl_pprop = values[2];
        this.br_pprop = values[3];

        this.intake_pprop = values[4];
        this.storage_1_pprop = values[5];

        if (values.length > 6) {
            this.storage_2_pprop = values[6];
            this.shooter1_pprop = values[7];
            this.shooter2_pprop = values[8];
            this.hang_l_pprop = values[9];
            this.hang_r_pprop = values[10];
            this.rin_prop = values[11];
        } else {
            this.storage_2_pprop = 0;
            this.shooter1_pprop = 0;
            this.shooter2_pprop = 0;
            this.hang_l_pprop = 0;
            this.hang_r_pprop = 0;
            this.rin_prop = 0;
        }
    }
}