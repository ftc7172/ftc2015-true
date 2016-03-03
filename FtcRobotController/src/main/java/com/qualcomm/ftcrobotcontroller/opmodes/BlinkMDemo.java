
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.util.Range;

public class BlinkMDemo extends LinearOpMode {
    BlinkM lblink;
    DualPad gpads;

    Toggle red = new Toggle();
    Toggle grn = new Toggle();
    Toggle blu = new Toggle();
    Toggle freq = new Toggle();

    public BlinkMDemo() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        lblink = new BlinkM(hardwareMap.i2cDevice.get("lblink"));
        gpads = new DualPad();
        waitOneFullHardwareCycle();
        while (time > -1) {
            gpads.setPads(gamepad1, gamepad2);
            if (gpads.right_trigger > 0) {
                lblink.setRGB(1,1,1);
                continue;
            }
            red.onRelease(gpads.b);
            grn.onRelease(gpads.a);
            blu.onRelease(gpads.x);
            freq.onRelease(gpads.y);
            double flash = (Math.cos(time * 3.14159 * (freq.count % 11) * 0.5) + 1)/2.0;
            lblink.setRGB((red.count % 6)/6.0 * flash,
                    (grn.count % 6)/6.0 * flash,
                    (blu.count % 6)/6.0 * flash);
            waitOneFullHardwareCycle();

        }
        waitForStart();
    }

}
