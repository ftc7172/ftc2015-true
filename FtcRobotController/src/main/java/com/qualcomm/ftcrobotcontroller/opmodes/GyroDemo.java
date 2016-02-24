
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

public class GyroDemo extends LinearOpMode {
    GyroSensor gyro;
    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor lb;
    DcMotor rb;

    public GyroDemo() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        leftWheel = hardwareMap.dcMotor.get("lf");
        leftWheel.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightWheel = hardwareMap.dcMotor.get("rf");
        rightWheel.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightWheel.setDirection(DcMotor.Direction.REVERSE);
        lb = hardwareMap.dcMotor.get("lb");
        lb.setPowerFloat();
        rb = hardwareMap.dcMotor.get("rb");
        rb.setDirection(DcMotor.Direction.REVERSE);
        rb.setPowerFloat();

        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        waitOneFullHardwareCycle();
        while (gyro.isCalibrating()) { waitOneFullHardwareCycle(); };
        waitForStart();

        // drive heading 0, power 0.2, distance 2000, 10 second timeout
        drive(45, 0.2, 2000, 100);
    }

    public void drive(int target, double power, int distance, double timeout)
            throws InterruptedException
    {
        double stopTime = time + timeout;
        int leftStart = leftWheel.getCurrentPosition();
        int rightStart = rightWheel.getCurrentPosition();

        leftWheel.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightWheel.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        lb.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rb.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        // basic driving loop -- run until timeout reached
        while (time < stopTime) {
            // exit loop if we've reached target distance
            if (Math.abs(leftWheel.getCurrentPosition() - leftStart)
                    + Math.abs(rightWheel.getCurrentPosition() - rightStart) > distance) break;

            // read current heading, convert to -179..+180 range
            int heading = gyro.getHeading();
            if (heading > 180) heading = heading - 360;
            telemetry.addData("heading", heading);

            int error = heading - target;
            double turn = error / 50.0;
            double leftPower = power - turn;
            double rightPower = power + turn;

            leftPower = Range.clip(leftPower, -1, 1);
            rightPower = Range.clip(rightPower, -1, 1);
            leftWheel.setPower(leftPower);
            lb.setPower(leftPower);
            rightWheel.setPower(rightPower);
            rb.setPower(rightPower);
            waitOneFullHardwareCycle();
        }

        // turn off driving motors
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }

}
