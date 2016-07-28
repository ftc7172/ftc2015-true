package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by robotics on 6/8/2016.
 */
public class ColorStop implements stopCondition {
    private int currentGreen;
    public ColorStop(){
        currentGreen=0;
    }
    @Override
    public boolean stop() {
        return currentGreen> EsBot.GREEN_CUTOFF;
    }

    @Override
    public void update(double newValue) {
        currentGreen=(int)newValue;
    }

    @Override
    public String type() {
        return "color";
    }
}
