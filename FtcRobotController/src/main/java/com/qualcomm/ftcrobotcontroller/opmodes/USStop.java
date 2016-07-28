package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by robotics on 6/8/2016.
 */
public class USStop implements stopCondition{
    private double sensor, iStopDistance;
    private double  CM_PER_UNIT=1;
    public USStop(double stopcm){
        iStopDistance=stopcm/CM_PER_UNIT;
        sensor=Integer.MAX_VALUE;
    }
    @Override
    public boolean stop() {
        return sensor<iStopDistance;
    }

    @Override
    public void update(double newValue) {
        sensor=(newValue+256)%256;
    }

    @Override
    public String type() {
        return "ultrasonic";
    }
}
