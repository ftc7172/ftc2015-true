package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by robotics on 2/24/2016.
 */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

public class SensorTestDIS extends LinearOpMode {
    public SensorTestDIS(){

    }

    public void runOpMode() throws InterruptedException{
        AnalogInput dist = hardwareMap.analogInput.get("dis");
        waitOneFullHardwareCycle();
      //  color.enableLed(true);
       // waitOneFullHardwareCycle();
        while(true) {
            telemetry.addData("OpDist: ", dist.getValue());
            waitOneFullHardwareCycle();
        }
    }
}
