package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by robotics on 1/5/2016.
 */
public class Toggle {
    public int count = 0;
    public boolean onoff = false;
    boolean buttonLast = false;

    public boolean onPress(boolean buttonNow) {
        if (buttonLast && !buttonNow) { count++; onoff = !onoff; }
        buttonLast = buttonNow;
        return onoff;
    }

    public boolean onRelease(boolean buttonNow) {
        if (!buttonLast && buttonNow) { count++; onoff = !onoff; }
        buttonLast = buttonNow;
        return onoff;
    }

    public boolean isOnoff() { return onoff; }
}

