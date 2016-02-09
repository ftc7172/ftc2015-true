package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by pmichaud on 2/8/2016.
 */
public class OhAuto extends LinearOpMode {
    DcMotor tiltMotor;
    DcMotor panMotor;


    double tiltTarget = 0;
    double panTarget = 0;
    GyroSensor gyro;

    Servo intakeServo;

    Toggle intakeToggle;

    DcMotor lf;
    DcMotor lb;
    DcMotor rf;
    DcMotor rb;


    public OhAuto() {
    }

    @Override
        public void runOpMode() throws InterruptedException{
        tiltMotor = hardwareMap.dcMotor.get("tilt");
        tiltMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        panMotor = hardwareMap.dcMotor.get("pan");
        gyro = hardwareMap.gyroSensor.get("gyro");
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
        gyro.calibrate();
        waitForStart();
        drive(30, .15, 0);
        }

    public void drive(double targetAngle, double throttle, double distance)throws InterruptedException{
        while(true) {
            double error = (gyro.getHeading () > 180 ? gyro.getHeading() - 360 : gyro.getHeading() - targetAngle) / (50.0);
            throttle = Range.clip(throttle, -1, 1);


            double right = throttle + error;
            double left = throttle - error;

            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);
            rf.setPower(right);
            rb.setPower(right);
            lf.setPower(left);
            lb.setPower(left);

            waitOneFullHardwareCycle();
            telemetry.addData("heading", gyro.getHeading());
            telemetry.addData("error", error);
        }
    }


    public void climberScore(){

    }

    public void lineFollow(){

    }


}
