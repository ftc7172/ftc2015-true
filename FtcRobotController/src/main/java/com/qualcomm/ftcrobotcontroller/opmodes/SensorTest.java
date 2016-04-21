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
        //OpticalDistanceSensor dist = hardwareMap.opticalDistanceSensor.get("dis");
        ColorSensor rcolor = hardwareMap.colorSensor.get("fcolor");
         rcolor.enableLed(false

         );
        waitOneFullHardwareCycle();
      //  color.enableLed(true);
       // waitOneFullHardwareCycle();
        while(true) {
           // telemetry.addData("OpDist: ", dist.getLightDetected())
           //;
            rcolor.enableLed(true);
            waitOneFullHardwareCycle();
            telemetry.addData("Blue: ", rcolor.blue());
            telemetry.addData("Red: ", rcolor.red());
            telemetry.addData("Green: ", rcolor.green());
            telemetry.addData("address: ", rcolor.getI2cAddress());
            waitOneFullHardwareCycle();
        }
    }
}
