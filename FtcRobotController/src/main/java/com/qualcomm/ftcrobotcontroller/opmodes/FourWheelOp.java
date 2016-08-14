package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by robotics on 8/7/2016.
 */
public class FourWheelOp extends OpMode {
    DcMotor lf;
    DcMotor lr;
    DcMotor rf;
    DcMotor rr;

    public void init() {
        lf = hardwareMap.dcMotor.get("lf");
        lr = hardwareMap.dcMotor.get("lr");
        rf = hardwareMap.dcMotor.get("rf");
        rr = hardwareMap.dcMotor.get("rr");
        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop() {
        double throttle = gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;

        double lpower = Range.clip(throttle + turn, -1, 1);
        double rpower = Range.clip(throttle-turn, -1, 1);

        lf.setPower(lpower);
        lr.setPower(lpower);
        rf.setPower(rpower);
        rr.setPower(rpower);
    }
}
