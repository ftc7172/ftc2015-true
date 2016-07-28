package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by robotics on 6/8/2016.
 */
public class DistanceStop implements stopCondition {
    private double travellNeed;
    public DistanceStop( double distance){
        travellNeed=distance;
    }
    @Override
    public boolean stop() {
        return travellNeed<=0;
    }

    @Override
    public void update(double newValue) {
        travellNeed-=newValue;
    }

    @Override
    public String type() {
        return "distance";
    }
}
