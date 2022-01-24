package frc.robot.sensors;

import java.util.Date;
import java.util.Calendar;
import frc.robot.state.MainState;

public abstract class BaseSensor {
    double LAG_TIME = 0;
    double last_updated = 0;
    Calendar self_timer = Calendar.getInstance();
    double SYNC_TIME = 0;

    MainState internal_state;

}