package frc.robot.network;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Map {
    NetworkTableEntry m_array;
    NetworkTableEntry m_width;
    NetworkTableEntry m_height;
    NetworkTableEntry m_origin_x;
    NetworkTableEntry m_origin_y;


    public void init(NetworkTableInstance inst) {
        NetworkTable stereo = inst.getTable("point");

        this.m_array = stereo.getEntry("m_array");
        this.m_array.setNumberArray(new Number[0]);

        this.m_width = stereo.getEntry("m_width");
        this.m_width.setNumber(0);

        this.m_height = stereo.getEntry("m_height");
        this.m_height.setNumber(0);

        this.m_origin_x = stereo.getEntry("m_origin_x");
        this.m_origin_x.setDouble(0);

        this.m_origin_y = stereo.getEntry("m_origin_y");
        this.m_origin_y.setDouble(0);
    }

    public int[][] getMap() {
        int[][] map = new int[m_width.getNumber(0).intValue()][m_height.getNumber(0).intValue()];
        for(int r = 0; r < map.length; r++) {
            for(int c = 0; c < map[0].length; c++) {
                map[r][c] = m_array.getNumberArray(new Number[0])[r * map[0].length + c].intValue();
            }
        }

        return map;
    }

    public double getMapOriginX() {
        return m_origin_x.getDouble(0);
    }

    public double getMapOriginY() {
        return m_origin_y.getDouble(0);
    }
}
