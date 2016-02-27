package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by robotics on 2/24/2016.
 */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

public class SensorTest extends LinearOpMode {
    public SensorTest(){

    }

    public void runOpMode() throws InterruptedException{
        OpticalDistanceSensor dist = hardwareMap.opticalDistanceSensor.get("dis");
        ColorSensor color = hardwareMap.colorSensor.get("color");
         color.enableLed(false);
        waitOneFullHardwareCycle();
      //  color.enableLed(true);
       // waitOneFullHardwareCycle();
        while(true) {
            telemetry.addData("OpDist: ", dist.getLightDetected());
            telemetry.addData("Blue: ", color.blue());
            telemetry.addData("Red: ", color.red());
            telemetry.addData("Green: ", color.green());
            telemetry.addData("address: ", color.getI2cAddress());
            waitOneFullHardwareCycle();
        }
    }
}
