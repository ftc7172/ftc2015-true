
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
public class EsAuto extends LinearOpMode {
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
    //Servo fenderl;
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
    Servo pole;

    I2cDevice myUS;
    I2cDeviceReader USreader;

    OpticalDistanceSensor opD;

    public EsAuto() {
    }

    public int heading() {
        return gyro.getHeading() > 180 ? gyro.getHeading() - 360 : gyro.getHeading();
    }

    @Override
    public void runOpMode()
            throws InterruptedException {
        dist = hardwareMap.analogInput.get("dis");
        // = hardwareMap.servo.get("lfender");
        fenderr = hardwareMap.servo.get("rfender");
        fenderUp();
        rZip = hardwareMap.servo.get("rzip");
        bZip = hardwareMap.servo.get("bzip");
        bZip.setPosition(Arbot.BZIP_UP);
        rZip.setPosition(Arbot.RZIP_UP);
        intakeServo = hardwareMap.servo.get("intake");
        intakeL = hardwareMap.servo.get("intake1");
        intakeServo.setPosition(0.5);
        intakeL.setPosition(0.5);
        fenderr = hardwareMap.servo.get("rfender");
        pole = hardwareMap.servo.get("pole");
        pole.setPosition(0.5);
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
        fcolor.enableLed(true);
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

        myUS = hardwareMap.i2cDevice.get("us");
        USreader = new I2cDeviceReader(myUS, 0x28, 0x04, 2);
        waitOneFullHardwareCycle();
        gyro.calibrate();
        while (true) {
            waitOneFullHardwareCycle();
            gpads.setPads(gamepad1, gamepad2);
            if (gpads.a||gpads.shift_a) break;
            delayToggle.onPress(gpads.b);
            endP.onPress(gpads.x);
            startP.onPress(gpads.y);
            if (!(gpads.shift_x || gpads.shift_b))
                blueTeam = rbswitch.isPressed();
            delaySeconds = (delayToggle.count % 16) * 2;
            startPos = startP.count % 2;
            endPos = endP.count % 4;
// in case of switch failure
            if (gpads.shift_x) {
                blueTeam = true;
            }
            if (gpads.shift_b) {
                blueTeam = false;
            }

            telemetry.addData("alliance", (blueTeam ? "blue" : "red"));
            telemetry.addData("delay (B)", delaySeconds + " seconds");
            telemetry.addData("start position (Y)", startPos);
            telemetry.addData("end position (X)", endPos);
            telemetry.addData("gyro", gyro.isCalibrating() ? "not ready" : "ready");
            telemetry.addData("ready (A)", "NO");
        }
        while (gyro.isCalibrating()) {
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
        if (blueTeam) {
            if (startPos == 0) {
                drive(0, -.35, new DistanceStop(3200));
                drive(35, -.35, new DistanceStop(8500));
            } else {
                drive(0, -.35, new DistanceStop(2400));
                drive(35, -.35, new DistanceStop(10000));
            }

            sleep(250);
            drive(30, .35, new DistanceStop(1500), new TimeStop(time, 5000));
            drive(100, -.13, new DistanceStop(1800), new ColorStop(), new TimeStop(time, 5000));

            // fenderUp();
            sleep(250);
            drive(heading(), -.12, new DistanceStop(1500), new USStop(13), new TimeStop(time, 3000));
            drive(85, 0.12, new DistanceStop(300), new TimeStop(time, 5000));//HTO
            drive(85, -0.12, new DistanceStop(6500), new USStop(13), new TimeStop(time, 3000));//hto
            drive(85, 0.12, new DistanceStop(200), new TimeStop(time,3000));//halved timeout
            fenderUp();
            sleep(250);
            drive(85, -0.12, new DistanceStop(325
            ), new TimeStop(time, 3000));

        } else {
            if (startPos == 0) {

            } else {

            }
            drive(0, -.35, new DistanceStop(3200));
            drive(-35, -.35, new DistanceStop(8500));

            sleep(250);
            drive(-30, .35, new DistanceStop(1500));
            drive(-100, -.13, new DistanceStop(1800), new ColorStop());

            //fenderUp();
            waitOneFullHardwareCycle();
            // fenderUp();
            waitOneFullHardwareCycle();
            sleep(250);
            drive(heading(), -.12, new DistanceStop(1500), new USStop(13), new TimeStop(time, 3000));
            drive(-85, 0.12, new DistanceStop(300), new TimeStop(time, 5000));//hto
            drive(-85, -0.12, new DistanceStop(6500), new USStop(14), new TimeStop(time,3000));//hto
            drive(-85, 0.12, new DistanceStop(200), new TimeStop(time, 3000));//hto
            fenderUp();
            sleep(250);
            drive(-85, -0.12, new DistanceStop(325), new TimeStop(time, 3000));

        }

        climberScore();

        if (endPos == 1) {
            if (blueTeam) {
                drive(85, 0.2, new DistanceStop(10000));
            } else {
                drive(-85, 0.2, new DistanceStop(10000));
            }
        } else if (endPos == 2) {
            if (blueTeam) {
                drive(45, 0.2, new DistanceStop(4500));
                drive(110, -0.35, new DistanceStop(7000));
                int lt = lb.getCurrentPosition();
                int rt = rb.getCurrentPosition();
                waitOneFullHardwareCycle();
                lb.setTargetPosition(lt);
                rb.setTargetPosition(rt);
                waitOneFullHardwareCycle();
            } else {
                drive(-45, 0.2, new DistanceStop(4500));
                drive(-110, -0.35, new DistanceStop(7000));
                int lt = lb.getCurrentPosition();
                int rt = rb.getCurrentPosition();
                waitOneFullHardwareCycle();
                lb.setTargetPosition(lt);
                rb.setTargetPosition(rt);
                waitOneFullHardwareCycle();
            }
        }
    }

    //This positions the arm to score the climbers}


    public boolean drive(double targetAngle, double throttle, stopCondition... stops) throws InterruptedException {
        int pastRPos = rb.getCurrentPosition();
        int pastLPos = lb.getCurrentPosition();
        double distanceShift = 0;
        byte USreading = Byte.MAX_VALUE;
        while (stops.length > 0) {
            double heading = heading();
            double error = (heading) - targetAngle;
            USreading = (byte) (USreader.getReadBuffer()[0] & 0xFF);
                telemetry.addData("distance", USreading);
            distanceShift = Math.cos(error * Math.PI / 180) * (Math.abs(rb.getCurrentPosition() - pastRPos) + Math.abs(lb.getCurrentPosition() - pastLPos));
            for (stopCondition s : stops) {
                if (s.type().equals("distance")) {
                    s.update(distanceShift);
                    if (s.stop()) {
                        rf.setPower(0);
                        rb.setPower(0);
                        lf.setPower(0);
                        lb.setPower(0);
                        waitOneFullHardwareCycle();
                        return false;
                    }
                }
                if (s.type().equals("time")) {
                    s.update(time);
                    if (s.stop()) {
                        rf.setPower(0);
                        rb.setPower(0);
                        lf.setPower(0);
                        lb.setPower(0);
                        waitOneFullHardwareCycle();
                        return true;
                    }
                }
                if (s.type().equals("color")) {
                    s.update(fcolor.green());
                    if (s.stop()) {
                        rf.setPower(0);
                        rb.setPower(0);
                        lf.setPower(0);
                        lb.setPower(0);
                        waitOneFullHardwareCycle();
                        return false;
                    }
                }
                if (s.type().equals("ultrasonic")) {
                    s.update(USreading);
                    if (s.stop()) {
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
            if (throttle < 0) {
                right = Range.clip(right, Math.max(2 * throttle, -1), 0);
                left = Range.clip(left, Math.max(2 * throttle, -1), 0);
            } else {
                right = Range.clip(right, 0, Math.min(2 * throttle, 1));
                left = Range.clip(left, 0, Math.min(2 * throttle, 1));
            }
            if (throttle > 0 || throttle < -0.2) {
                rf.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                rb.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                lf.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                lb.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            } else {
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

        // fenderl.setPosition(Arbot.LFENDER_DOWN);
        fenderr.setPosition(Arbot.RFENDER_DOWN);
    }


    public void fenderUp() {
        //fenderl.setPosition(Arbot.LFENDER_UP);
        fenderr.setPosition(Arbot.RFENDER_UP);
    }

    public void climberScore() throws InterruptedException {
        tiltUpArm(1690);
        sleep(1000);
        telemetry.addData("lcolor.red", lcolor.red());
        telemetry.addData("lcolor.blue", lcolor.blue());
        telemetry.addData("rcolor.red", rcolor.red());
        telemetry.addData("rcolor.blue", rcolor.blue());
        waitOneFullHardwareCycle();
        if ((blueTeam && rcolor.blue() > rcolor.red())
                || (!blueTeam && rcolor.red() > rcolor.blue())) {
            panRight(-100, new TimeStop(time, 3000));
            drive(heading(), -0.15, new DistanceStop(250));
            sleep(250);
            drive(heading(), 0.3, new DistanceStop(500));
            panLeft(0, new TimeStop(time, 3000));
        } else if ((blueTeam && lcolor.blue() > lcolor.red())
                || (!blueTeam && lcolor.red() > lcolor.blue())) {
            panLeft(100, new TimeStop(time, 3000));
            drive(heading(), -0.15, new DistanceStop(250));
            sleep(250);
            drive(heading(), 0.3, new DistanceStop(500));
            panRight(0, new TimeStop(time, 3000));
        }
        tiltDownArm(300);
        tiltMotor.setPowerFloat();
    }

    public void tiltUpArm(double t) throws InterruptedException {
        tiltTarget = t;
        telemetry.addData("tiltTarget", tiltTarget);
        while (true) {
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
            tiltMotor.setPower(-.2);
            waitOneFullHardwareCycle();
        }
        tiltMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        tiltMotor.setPower(0);
        waitOneFullHardwareCycle();
    }

    public void panRight(int panTarget, TimeStop t) throws InterruptedException {


        panMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        double panPos = panMotor.getCurrentPosition();
        while (true) {
            panPos = panMotor.getCurrentPosition();
            if (panPos < panTarget) {
                break;
            }
            if(t.stop()) break;
            panMotor.setPower(-.18);
            waitOneFullHardwareCycle();
        }
        panMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        panMotor.setPower(0);
        waitOneFullHardwareCycle();
    }

    public void panLeft(int panTarget, TimeStop t) throws InterruptedException {


        panMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        double panPos = panMotor.getCurrentPosition();
        while (true) {
            panPos = panMotor.getCurrentPosition();
            if (panPos > panTarget) {
                break;
            }
            if(t.stop()) break;
            panMotor.setPower(0.18);
            waitOneFullHardwareCycle();
        }
        panMotor.setPower(0);
        waitOneFullHardwareCycle();
    }

}
