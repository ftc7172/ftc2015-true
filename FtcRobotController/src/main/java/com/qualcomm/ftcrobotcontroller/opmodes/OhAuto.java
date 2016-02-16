package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by pmichaud on 2/8/2016.
 */
public class OhAuto extends LinearOpMode {
    DcMotor tiltMotor;
   // DcMotor panMotor;

    DualPad gpads;

    double tiltTarget = 0;

    GyroSensor gyro;
    DcMotor extendMotor;
    Servo rZip;
    Servo bZip;
    Servo intakeServo;


    DcMotor lf;
    DcMotor lb;
    DcMotor rf;
    DcMotor rb;

    TouchSensor rbswitch;
    boolean blueTeam;

    Toggle delayToggle;
    int delaySeconds;

    OpticalDistanceSensor opD;


    public OhAuto() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        tiltMotor = hardwareMap.dcMotor.get("tilt");
        tiltMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        tiltMotor.setDirection(DcMotor.Direction.REVERSE);
      //  panMotor = hardwareMap.dcMotor.get("pan");
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
        gpads = new DualPad();
        rbswitch = hardwareMap.touchSensor.get("rbswitch");
        delayToggle = new Toggle();
        rZip = hardwareMap.servo.get("rzip");
        bZip = hardwareMap.servo.get("bzip");
        intakeServo = hardwareMap.servo.get("intake");
        bZip.setPosition(0);
        rZip.setPosition(1);
        intakeServo.setPosition(0.5);
        gyro.calibrate();
        while (true) {
            waitOneFullHardwareCycle();
            gpads.setPads(gamepad1, gamepad2);
            if (gpads.a) break;
            delayToggle.onPress(gpads.b);

            blueTeam = rbswitch.isPressed();
            delaySeconds = (delayToggle.count % 7) * 5;

            telemetry.addData("alliance", (blueTeam ? "blue" : "red"));
            telemetry.addData("delay (B)", delaySeconds + " seconds");
            telemetry.addData("gyro", gyro.isCalibrating() ? "not ready" : "ready");
            telemetry.addData("ready (A)", "NO");
        }
        while(gyro.isCalibrating()){
            waitOneFullHardwareCycle();
        }
        boolean score = true;
        telemetry.addData("ready (A)", "yes");
        waitForStart();
        sleep(delaySeconds * 1000);
        extendMotor.setPowerFloat();
        //This drives our robot to the beacon
        if(blueTeam){
            score = drive(0, -.5, 8600, 5);
            score = drive(45, -.5, 15000, 10) && score;
            score = drive(85, -.5, 4000, 5) && score;
            drive(85, -.15, 6400, 3);
            drive(85, .15, 400, 3);
            if(score) climberScore();
            drive(85, .15, 12800, 5);
        }
        else {
            score = drive(0, -.5, 8500, 5);
            score = drive(-45, -.5, 15000, 10) && score;
            score = drive(-85, -.5, 4000, 5) && score;
            drive(-85, -.15, 6400, 3);
            drive(-85, .15, 400, 3);
            if(score) climberScore();
            drive(-85, .15, 12800, 5);
        }
        //This positions the arm to score the climbers



        }

    public boolean drive(double targetAngle, double throttle, double distance, double timeout)throws InterruptedException{
        int startRPos = rf.getCurrentPosition();
        int startLPos = lf.getCurrentPosition();
        double startTime = time;
        while (Math.abs(rf.getCurrentPosition() - startRPos) + Math.abs(lf.getCurrentPosition()-startLPos)< distance) {
            if(time > startTime + timeout) {
                break;
            }
            telemetry.addData("lf", lf.getCurrentPosition()-startLPos);
            //The gyro sensor allows the robot to drive in a straight line regardless of debris
            double error = ((gyro.getHeading() > 180 ? gyro.getHeading() - 360 : gyro.getHeading()) - targetAngle) / (50.0);
            throttle = Range.clip(throttle, -1, 1);
            double right = throttle + error;
            double left = throttle - error;
            if(throttle < 0) {
                right = Range.clip(right, -1, 0);
                left = Range.clip(left, -1, 0);
            }
            else{
                right = Range.clip(right, -1, 1);
                left = Range.clip(left, -1, 1);
            }

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
            waitOneFullHardwareCycle();
        }
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
        waitOneFullHardwareCycle();
        return (time < startTime + timeout);
    }


    public void climberScore() throws InterruptedException{
        tiltUpArm(1750);
        sleep(1000);
        tiltDownArm(300);
        tiltMotor.setPowerFloat();
    }

    public void tiltUpArm(double t) throws InterruptedException{
        tiltTarget = t;
        telemetry.addData("tiltTarget", tiltTarget);
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

    public void tiltDownArm(double t) throws InterruptedException {
        tiltTarget = t;
        telemetry.addData("tiltTarget", tiltTarget);
        while (true) {
            double tiltPos = tiltMotor.getCurrentPosition();
            if (tiltPos < tiltTarget) {
                break;
            }
            tiltMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            tiltMotor.setPower(- .2);
            waitOneFullHardwareCycle();
        }
        tiltMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        tiltMotor.setPower(0);
        waitOneFullHardwareCycle();
    }


}
