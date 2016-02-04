package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by robotics on 1/5/2016.
 */
public class Toggle {
    boolean onoff = false;
    boolean buttonLast = false;

    public boolean onRelease(boolean buttonNow) {
        if (buttonLast && !buttonNow) onoff = !onoff;
        buttonLast = buttonNow;
        return onoff;
    }

    public boolean isOnoff() { return onoff; }
}


