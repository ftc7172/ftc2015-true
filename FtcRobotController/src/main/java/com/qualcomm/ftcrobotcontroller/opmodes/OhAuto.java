package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
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
    double traveled = 0;
    double pastPos = 0;
    GyroSensor gyro;
    DcMotor extendMotor;
    Servo intakeServo;

    Toggle intakeToggle;

    DcMotor lf;
    DcMotor lb;
    DcMotor rf;
    DcMotor rb;

    OpticalDistanceSensor opD;


    public OhAuto() {
    }

    @Override
        public void runOpMode() throws InterruptedException{
        tiltMotor = hardwareMap.dcMotor.get("tilt");
        tiltMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        tiltMotor.setDirection(DcMotor.Direction.REVERSE);
        panMotor = hardwareMap.dcMotor.get("pan");
        gyro = hardwareMap.gyroSensor.get("gyro");
        lf = hardwareMap.dcMotor.get("lf");
        lf.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        extendMotor = hardwareMap.dcMotor.get("extend");
        extendMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
       // opD = hardwareMap.opticalDistanceSensor.get("opdist");
        rf.setDirection(DcMotor.Direction.REVERSE);
        rf.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rb.setDirection(DcMotor.Direction.REVERSE);
        gyro.calibrate();
        while(gyro.isCalibrating()){
            waitOneFullHardwareCycle();
        }
        waitForStart();
        extendMotor.setPowerFloat();
        drive(0, -.2, 4200);
        drive(-45, -.2, 7500);
        drive(-80, -.2, 500);
        climberScore();
        /*
        drive(0, .15, 50)
        while(opD.getLightDetected() < "Red"){
            drive(-30, .15, 10);
        }
        while(opD.getLightDetected < "white"){
            drive(0, .15, 10);
        }
        drive(-90, .15, 10);
        lineFollow();
        scoreClimbers();
        */

        }

    public void drive(double targetAngle, double throttle, double distance)throws InterruptedException{

        int startPos = rf.getCurrentPosition();
        while (Math.abs(rf.getCurrentPosition() - startPos) < distance) {
            double error = ((gyro.getHeading() > 180 ? gyro.getHeading() - 360 : gyro.getHeading()) - targetAngle) / (50.0);
            throttle = Range.clip(throttle, -1, 1);
            double right = throttle + error;
            double left = throttle - error;

            right = Range.clip(right, -1, 0);
            left = Range.clip(left, -1, 0);
            rf.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            rb.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            lf.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            lb.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

            rf.setPower(right);
            rb.setPower(right);
            lf.setPower(left);
            lb.setPower(left);

            telemetry.addData("rfpos", rf.getCurrentPosition());

            telemetry.addData("heading", gyro.getHeading());
            telemetry.addData("error", error);
            //telemetry.addData("color", opD.getLightDetected());
            waitOneFullHardwareCycle();
        }
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
        waitOneFullHardwareCycle();
    }


    public void climberScore() throws InterruptedException{
        tiltArm(1700);
       // tiltArm(100);
    }

    public void tiltArm(double t) throws InterruptedException{
        int PID_RANGE = 50;
        tiltTarget = t;
        telemetry.addData("tiltTarget", tiltTarget);
        //telemetry.addData("tiltPos", tiltPos);
        while(true) {
            double tiltPos = tiltMotor.getCurrentPosition();
            if (tiltPos > tiltTarget) {
                break;
            }
            tiltMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            tiltMotor.setPower(.2);
            waitOneFullHardwareCycle();
        }
        tiltMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        tiltMotor.setPower(0);
        waitOneFullHardwareCycle();
    }

 /*   public void lineFollow(double lnColor) throws InterruptedException{
        double i = 0;
        while(true){
            double light = opD.getLightDetected();
            while(light < lnColor) {
                i = 0;
                drive(0, .15, 10);
                light = opD.getLightDetected();
            }
            i+= .1;
            double d = Math.sin(i);
            drive(d, .15, 10);
        }

    }
*/

}
