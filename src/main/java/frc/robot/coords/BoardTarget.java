package frc.robot.coords;

public class BoardTarget {
    public static final double deg2rad = 2 * Math.PI / 360;

    //35.875 in from hub to fender, 2.959m  
    public static final double[] R1 = {2.9 * 0.358, 2.9 * 0.934, -21 * deg2rad};
    public static final double[] R2 = {1.937, -1.039, -111 * deg2rad};
    public static final double[] B3 = {3.508, -2.707, 159 * deg2rad};
    public static final double[] B4 = {5.078, 1.039, 69 * deg2rad};
}
