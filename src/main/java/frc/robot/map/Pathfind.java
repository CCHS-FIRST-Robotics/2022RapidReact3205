package frc.robot.map;

import frc.robot.network.Network;

public class Pathfind {

    int[][] map;
    double originX;
    double originY;

    public Pathfind(Network network) {
        this.map = network.map.getMap().clone();
        originX = network.map.getMapOriginX();
        originY = network.map.getMapOriginY();
    }
}
