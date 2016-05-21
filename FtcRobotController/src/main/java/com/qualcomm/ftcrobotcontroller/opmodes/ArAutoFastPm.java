
package com.qualcomm.ftcrobotcontroller.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by pmichaud on 2/8/2016.
 */
public class ArAutoFastPm extends LinearOpMode {
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

    public ArAutoFastPm() {
    }

    public int heading() {
        return gyro.getHeading() > 180 ? gyro.getHeading() - 360: gyro.getHeading();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        dist = hardwareMap.analogInput.get("dis");
        fenderl = hardwareMap.servo.get("lfender");
        fenderr = hardwareMap.servo.get("rfender");
        fenderUp();
        rZip = hardwareMap.servo.get("rzip");
        bZip = hardwareMap.servo.get("bzip");
        bZip.setPosition(Arbot.BZIP_UP);
        rZip.setPosition(Arbot.RZIP_UP);
        intakeServo = hardwareMap.servo.get("intake");
        intakeL= hardwareMap.servo.get("intake1");
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
            endPos = endP.count % 4;

            telemetry.addData("alliance", (blueTeam ? "blue" : "red"));
            telemetry.addData("delay (B)", delaySeconds + " seconds");
            telemetry.addData("start position (Y)", startPos);
            telemetry.addData("end position (X)", endPos);
            telemetry.addData("gyro", gyro.isCalibrating() ? "not ready" : "ready");
            telemetry.addData("ready (A)", "NO");
        }
        while(gyro.isCalibrating()){
            waitOneFullHardwareCycle();
        }
        boolean score = true;
        telemetry.addData("ready (A)", "yes");
        waitForStart();
        gyro.resetZAxisIntegrator();
        sleep(delaySeconds * 1000);
        extendMotor.setPowerFloat();

        //This drives our robot to the beacon
        fenderDown();
        fcolor.enableLed(true);
        if(blueTeam){
            if(startPos == 0) {
                drive(0, -.6, 3200, 10, false);
                drive(35, -.6, 8500, 10, false);
            }
            else{
                drive(0, -.35, 2400, 10, false);
                drive(35, -.35, 10000, 10, false);
            }
            sleep(250);
            drive(30, .6, 1800, 10, false);
            drive(100, -.13, 1800, 10, true);
            fenderUp();
            sleep(250);
            drive(heading(),-.12,1500,10,false);
            drive(80,0.12,300,5,false);//HTO
            drive(80,-0.12,6500,1.5,false);//hto
            drive(80,0.12,100,5,false);//halved timeout
        } else {
            if(startPos == 0){

            }
            else{

            }
            drive(0, -.6, 3200, 10, false);
            //sleep(250);
            drive(-35, -.6, 8500, 10, false);
            sleep(250);
            drive(-30, .6, 1800, 10, false);
            drive(-100, -.13, 1800, 10, true);
            fenderUp();
            waitOneFullHardwareCycle();
            fenderUp();
            waitOneFullHardwareCycle();
            sleep(250);
            drive(heading(),-.12,1500,10,false);
            drive(-80,0.12,300,5,false);//hto
            drive(-80, -0.12, 6500, 1.5, false);//hto
            drive(-80,0.12,100,5,false);//hto
        }

        climberScore();

        if(endPos == 1){
            if(blueTeam){
                drive(85, 0.2, 10000, 10, false);
            }
            else{
                drive(-85, 0.2, 10000, 10, false);
            }
        }
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        else if(endPos ==2 ){
            if(blueTeam){
                /*drive(45, 0.2, 4500, 5, false);
                drive(110, -0.35, 7000, 10, false);
                int lt = lb.getCurrentPosition();
                int rt = rb.getCurrentPosition();
                waitOneFullHardwareCycle();
                lb.setTargetPosition(lt);
                rb.setTargetPosition(rt);
                waitOneFullHardwareCycle();
                */
                ///drive(angle (positive), power )
                drive(125, .5, 7000, 10, false);

            }

            else{
              /*  drive(-45, 0.2, 4500, 5, false);
                drive(-110, -0.35, 7000, 10, false);
                int lt = lb.getCurrentPosition();
                int rt = rb.getCurrentPosition();
                waitOneFullHardwareCycle();
                lb.setTargetPosition(lt);
                rb.setTargetPosition(rt);
                waitOneFullHardwareCycle(); */
                drive(-45, .5, 7000, 10, false);
            }
        }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


        //This positions the arm to score the climbers

    }

    public boolean drive(double targetAngle, double throttle, double distance, double timeout, boolean colorStop)throws InterruptedException{
        int pastRPos = rb.getCurrentPosition();
        int pastLPos = lb.getCurrentPosition();
        double startTime = time;
        double traveled = distance;
        while (time < startTime + timeout){
            double heading = gyro.getHeading() > 180 ? gyro.getHeading() - 360: gyro.getHeading();
            double error = (heading) - targetAngle;

            traveled -= Math.cos(error * Math.PI/180) * (Math.abs(rb.getCurrentPosition() - pastRPos) + Math.abs(lb.getCurrentPosition()-pastLPos));

            if (traveled <= 0 ){
                break;
            }
            pastRPos = rb.getCurrentPosition();
            pastLPos = lb.getCurrentPosition();

            if(colorStop && fcolor.green()>5) break;

            throttle = Range.clip(throttle, -1, 1);
            double right = throttle + error * .01;//.018
            double left = throttle - error * .01;
            if(throttle < 0) {
                right = Range.clip(right, Math.max(2*throttle, -1), 0);
                left = Range.clip(left, Math.max(2*throttle, -1), 0);
            }
            else{
                right = Range.clip(right, 0, Math.min(2*throttle, 1));
                left = Range.clip(left, 0, Math.min(2*throttle, 1));
            }
            if(throttle >  0 || throttle < -0.2) {
                rf.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                rb.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                lf.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                lb.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            }
            else{
                rf.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                rb.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                lf.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                lb.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            }

            rf.setPower(right);
            rb.setPower(right);
            lf.setPower(left);
            lb.setPower(left);
            waitOneFullHardwareCycle();
        }

        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
        waitOneFullHardwareCycle();
        return (time < startTime + timeout);
    }

    public void fenderDown() {

            fenderl.setPosition(Arbot.LFENDER_DOWN);
            fenderr.setPosition(Arbot.RFENDER_DOWN);
    }


    public void fenderUp() {
        fenderl.setPosition(Arbot.LFENDER_UP);
        fenderr.setPosition(Arbot.RFENDER_UP);
    }

    public void climberScore() throws InterruptedException {
        tiltUpArm(1625);
        sleep(1000);
        telemetry.addData("lcolor.red", lcolor.red());
        telemetry.addData("lcolor.blue", lcolor.blue());
        telemetry.addData("rcolor.red", rcolor.red());
        telemetry.addData("rcolor.blue", rcolor.blue());
        waitOneFullHardwareCycle();
        if ( (blueTeam && rcolor.blue()>rcolor.red())
                || (!blueTeam && rcolor.red()>rcolor.blue())) {
            panRight(-200);
            drive(heading(), -0.15, 250, 1, false);
            sleep(250);
            drive(heading(), 0.3, 500, 3, false);
            panLeft(0);
        }
        else if ( (blueTeam && lcolor.blue()>lcolor.red())
                || (!blueTeam && lcolor.red()>lcolor.blue())) {
            panLeft(200);
            drive(heading(), -0.15, 250, 1, false);
            sleep(250);
            drive(heading(), 0.3, 500, 3, false);
            panRight(0);
        }
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

    public void panRight(int panTarget) throws InterruptedException {
        double panPos = panMotor.getCurrentPosition();

        panMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        while(true) {
            panPos = panMotor.getCurrentPosition();
            if (panPos < panTarget) {
                break;
            }
            panMotor.setPower(-.18);
            waitOneFullHardwareCycle();
        }
        panMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        panMotor.setPower(0);
        waitOneFullHardwareCycle();
    }

    public void panLeft(int panTarget) throws InterruptedException {
        double panPos = panMotor.getCurrentPosition();

        panMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        while(true) {
            panPos = panMotor.getCurrentPosition();
            if (panPos > panTarget) {
                break;
            }
            panMotor.setPower(0.18);
            waitOneFullHardwareCycle();
        }
        panMotor.setPower(0);
        waitOneFullHardwareCycle();
    }

}
