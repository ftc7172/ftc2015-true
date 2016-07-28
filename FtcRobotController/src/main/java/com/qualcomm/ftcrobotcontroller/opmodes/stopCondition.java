package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by robotics on 6/8/2016.
 */
public interface stopCondition {
    boolean stop();
    void update(double newValue);
    String type();
}
