package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by robotics on 6/8/2016.
 */
public class TimeStop implements stopCondition {
    public double iTime, iEndTime;
    public TimeStop(double startTime, double duration){
        iTime=startTime;
        iEndTime=(duration/1000)+startTime;
    }
    @Override
    public boolean stop() {
       // telemetry.addData("end time", iEndTime);
        return iTime >= iEndTime;
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
