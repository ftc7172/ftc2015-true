package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by robotics on 3/27/2016.
 */
public class PresOp extends LinearOpMode {


    DcMotor tiltMotor;
    DcMotor panMotor;

    DualPad gpads;

    double tiltTarget = 0;

    GyroSensor gyro;
    DcMotor extendMotor;
    Servo rZip;
    Servo bZip;
    Servo intakeServo;
    Servo intakeL;


    DcMotor lf;
    DcMotor lb;
    DcMotor rf;
    DcMotor rb;
    AnalogInput dist;
    Servo fenderl;
    Servo fenderr;

    TouchSensor armTouch;
    TouchSensor rbswitch;
    Toggle startP;
    Toggle endP;
    boolean blueTeam;
    int startPos;
    int endPos;
    ColorSensor fcolor;
    ColorSensor rcolor;
    ColorSensor lcolor;


    Toggle delayToggle;
    int delaySeconds;

    OpticalDistanceSensor opD;

    public PresOp() {
    }

    public int heading() {
        return gyro.getHeading() > 180 ? gyro.getHeading() - 360: gyro.getHeading();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        dist = hardwareMap.analogInput.get("dis");
        fenderl = hardwareMap.servo.get("lfender");
        fenderr = hardwareMap.servo.get("rfender");
        rZip = hardwareMap.servo.get("rzip");
        bZip = hardwareMap.servo.get("bzip");
        bZip.setPosition(Arbot.BZIP_UP);
        rZip.setPosition(Arbot.RZIP_UP);
        intakeServo = hardwareMap.servo.get("intake");
        intakeL = hardwareMap.servo.get("intake1");
        armTouch = hardwareMap.touchSensor.get("ezero");
        intakeServo.setPosition(0.5);
        intakeL.setPosition(0.5);
        waitOneFullHardwareCycle();

        tiltMotor = hardwareMap.dcMotor.get("tilt");
        tiltMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        tiltMotor.setDirection(DcMotor.Direction.REVERSE);
        panMotor = hardwareMap.dcMotor.get("pan");
        panMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        panMotor.setDirection(DcMotor.Direction.REVERSE);
        gyro = hardwareMap.gyroSensor.get("gyro");

        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
        rb.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        lb.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        extendMotor = hardwareMap.dcMotor.get("extend");
        extendMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);


        fcolor = hardwareMap.colorSensor.get("fcolor");
        fcolor.enableLed(false);
        lcolor = hardwareMap.colorSensor.get("lcolor");
        lcolor.setI2cAddress(0x3a);
        lcolor.enableLed(false);
        rcolor = hardwareMap.colorSensor.get("rcolor");
        rcolor.setI2cAddress(0x3e);
        rcolor.enableLed(false);
        gpads = new DualPad();
        rbswitch = hardwareMap.touchSensor.get("rbswitch");
        delayToggle = new Toggle();
        startP = new Toggle();
        endP = new Toggle();
        opD = hardwareMap.opticalDistanceSensor.get("opdist");
        opD.enableLed(true);
        waitOneFullHardwareCycle();
        gyro.calibrate();
        while (true) {
            waitOneFullHardwareCycle();
            gpads.setPads(gamepad1, gamepad2);
            if (gpads.a) break;
            delayToggle.onPress(gpads.b);
            endP.onPress(gpads.x);
            startP.onPress(gpads.y);

            blueTeam = rbswitch.isPressed();
            delaySeconds = (delayToggle.count % 16) * 2;
            startPos = startP.count % 2;
            endPos = endP.count % 2;

            telemetry.addData("alliance", (blueTeam ? "blue" : "red"));
            telemetry.addData("delay (B)", delaySeconds + " seconds");
            telemetry.addData("start position (Y)", startPos);
            telemetry.addData("end position (X)", endPos);
            //telemetry.addData("gyro", gyro.isCalibrating() ? "not ready" : "ready");
            telemetry.addData("ready (A)", "NO");
        }
       /* while (gyro.isCalibrating()) {
            waitOneFullHardwareCycle();
*/        boolean score = true;
        telemetry.addData("ready (A)", "yes");
        waitForStart();
        sleep(delaySeconds * 1000);


        while(true){
            waitOneFullHardwareCycle();
            gpads.setPads(gamepad1, gamepad2);
            intakeL.setPosition(1);
            intakeServo.setPosition(0);
            waitOneFullHardwareCycle();
            if(!gpads.x) break;
            waitOneFullHardwareCycle();
        }
        intakeL.setPosition(.5);
        intakeServo.setPosition(.5);
        extend(1250);
        tiltMotor.setPowerFloat();
        waitOneFullHardwareCycle();
        sleep(1500);
        extendMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        tiltMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle();
        waitOneFullHardwareCycle();
        waitOneFullHardwareCycle();
        waitOneFullHardwareCycle();

        depBlock(0, 25);
        extendMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        tiltMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle();
        waitOneFullHardwareCycle();
        waitOneFullHardwareCycle();
        waitOneFullHardwareCycle();
        depBlock(-200, 100);
        depBlock(-400, 180);
        depBlock(-500, 260);
        depBlock(-550, 330);
        depBlock(-1400, 330);
        waitOneFullHardwareCycle();
        tiltMotor.setPowerFloat();
        waitOneFullHardwareCycle();
    }


    private void extend(int extendTarget) throws InterruptedException{
        extendMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        waitOneFullHardwareCycle();
        extendMotor.setPower(.2);
        extendMotor.setTargetPosition(extendTarget);
        waitOneFullHardwareCycle();
        while(extendMotor.isBusy()){
            telemetry.addData("extend: ", extendMotor.getCurrentPosition());
            waitOneFullHardwareCycle();
            boolean zero = armTouch.isPressed();
            waitOneFullHardwareCycle();
            if(zero){
                extendMotor.setPower(.5);
                waitOneFullHardwareCycle();
                return;
            }
        }
    }

    private void depBlock(int extPos, int tiltPos) throws InterruptedException{
        extend(extPos);
        tilt(tiltPos);
        waitOneFullHardwareCycle();
        intakeL.setPosition(.2);
        intakeServo.setPosition(.8);
       // waitOneFullHardwareCycle();
        sleep(320);
       // waitOneFullHardwareCycle();
        intakeL.setPosition(.5);
        intakeServo.setPosition(.5);
      //  waitOneFullHardwareCycle();
        sleep(500);
       // waitOneFullHardwareCycle();
    }

    private void tilt(int tiltTarget)throws InterruptedException{
        tiltMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        waitOneFullHardwareCycle();
        tiltMotor.setPower(.2);
        tiltMotor.setTargetPosition(tiltTarget);
        waitOneFullHardwareCycle();
        while(tiltMotor.isBusy()){
            waitOneFullHardwareCycle();
        }
    }



    }
