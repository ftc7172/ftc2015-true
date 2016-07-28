
package com.qualcomm.ftcrobotcontroller.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by pmichaud on 2/8/2016.
 */
public class EsUSTest extends LinearOpMode {
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
    I2cDevice myUS;
    I2cDeviceReader USreader;


    Toggle delayToggle;
    int delaySeconds;

    OpticalDistanceSensor opD;

    public EsUSTest() {
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
        myUS=hardwareMap.i2cDevice.get("us");
        USreader= new I2cDeviceReader(myUS,0x28,0x04,2);

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

        while(gyro.isCalibrating()){
            telemetry.addData("gyro", gyro.isCalibrating() ? "not ready" : "ready");
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
        telemetry.addData("heading",heading());
        byte usReading=0;
        while(true){
            usReading=(byte) (USreader.getReadBuffer()[0]&0xFF);
            telemetry.addData("US",usReading);
        }
        //drive(0, -0.2, new USStop(0), new DistanceStop(300));


        //This positions the arm to score the climbers

    }

    public boolean drive(double targetAngle, double throttle, stopCondition ... stops )throws InterruptedException{
        int pastRPos = rb.getCurrentPosition();
        int pastLPos = lb.getCurrentPosition();
        double distanceShift=0;
        byte USreading=0;

        while (stops.length>0){
            double heading = heading();
            double error = (heading) - targetAngle;
            USreading=(byte) (USreader.getReadBuffer()[0]&0xFF);

            distanceShift=Math.cos(error * Math.PI/180) * (Math.abs(rb.getCurrentPosition() - pastRPos) + Math.abs(lb.getCurrentPosition()-pastLPos));

            telemetry.addData("US",USreading);
            telemetry.addData("heading",heading);
            for(stopCondition s : stops){
                telemetry.addData("type",s.type());
                if(s.type().equals("distance")){
                    s.update(distanceShift);
                    if(s.stop())  {
                        rf.setPower(0);
                        rb.setPower(0);
                        lf.setPower(0);
                        lb.setPower(0);
                        waitOneFullHardwareCycle();
                        return false;
                    }
                }
                if(s.type().equals("time")){
                    s.update(time);
                    if(s.stop()){
                        rf.setPower(0);
                        rb.setPower(0);
                        lf.setPower(0);
                        lb.setPower(0);
                        waitOneFullHardwareCycle();
                        return true;
                    }
                }
                if(s.type().equals("color")){
                    s.update(fcolor.green());
                    if(s.stop()){
                        rf.setPower(0);
                        rb.setPower(0);
                        lf.setPower(0);
                        lb.setPower(0);
                        waitOneFullHardwareCycle();
                        return false;
                    }
                }
                if(s.type().equals("ultrasonic")){
                    s.update(USreading);
                    if(s.stop()) {
                        rf.setPower(0);
                        rb.setPower(0);
                        lf.setPower(0);
                        lb.setPower(0);
                        waitOneFullHardwareCycle();
                        return false;
                    }
                }

            }

            pastRPos = rb.getCurrentPosition();
            pastLPos = lb.getCurrentPosition();



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
        return false;
    }

    public void fenderDown() {

            fenderl.setPosition(Arbot.LFENDER_DOWN);
            fenderr.setPosition(Arbot.RFENDER_DOWN);
    }


    public void fenderUp() {
        fenderl.setPosition(Arbot.LFENDER_UP);
        fenderr.setPosition(Arbot.RFENDER_UP);
    }

   /* public void climberScore() throws InterruptedException {
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
    }*/

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
