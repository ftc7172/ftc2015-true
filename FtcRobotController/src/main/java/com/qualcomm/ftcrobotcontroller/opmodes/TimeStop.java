package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by robotics on 6/8/2016.
 */
public class TimeStop implements stopCondition {
    private double iTime, iEndTime;
    public TimeStop(double startTime, double duration){
        iTime=startTime;
        iEndTime=duration+startTime;
    }
    @Override
    public boolean stop() {
        return iTime>=iEndTime;
    }

    @Override
    public void update(double newValue) {
    iTime=newValue;
}

    @Override
    public String type() {
        return "time";
    }
}
