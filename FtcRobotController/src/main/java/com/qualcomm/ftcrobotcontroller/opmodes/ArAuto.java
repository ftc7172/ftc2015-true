
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
 * Created by pmichaud on 2/8/2016.
 */
public class ArAuto extends LinearOpMode {
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
    ColorSensor rcolor;


    Toggle delayToggle;
    int delaySeconds;


    BlinkM lblink;
    OpticalDistanceSensor opD;


    public ArAuto() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        dist = hardwareMap.analogInput.get("dis");
        fenderl = hardwareMap.servo.get("lfender");
        fenderr = hardwareMap.servo.get("rfender");
        fenderUp();
        tiltMotor = hardwareMap.dcMotor.get("tilt");
        tiltMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        tiltMotor.setDirection(DcMotor.Direction.REVERSE);
        panMotor = hardwareMap.dcMotor.get("pan");
        panMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
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

        lblink = new BlinkM(hardwareMap.i2cDevice.get("lblink"));
        lblink.setRGB(0, 1, 0);

        rcolor = hardwareMap.colorSensor.get("rcolor");
        rcolor.enableLed(false);
        gpads = new DualPad();
        rbswitch = hardwareMap.touchSensor.get("rbswitch");
        delayToggle = new Toggle();
        startP = new Toggle();
        endP = new Toggle();
        rZip = hardwareMap.servo.get("rzip");
        bZip = hardwareMap.servo.get("bzip");
        opD = hardwareMap.opticalDistanceSensor.get("opdist");
        opD.enableLed(true);
        intakeServo = hardwareMap.servo.get("intake");
        intakeL= hardwareMap.servo.get("intake1");
        bZip.setPosition(.2);
        rZip.setPosition(.85);
        intakeServo.setPosition(0.5);
        intakeL.setPosition(0.5);
        waitOneFullHardwareCycle();
        gyro.calibrate();
        while (true) {
            waitOneFullHardwareCycle();
            lblink.setRGB(0, 1, 0);
            gpads.setPads(gamepad1, gamepad2);
            if (gpads.a) break;
            delayToggle.onPress(gpads.b);
            endP.onPress(gpads.x);
            startP.onPress(gpads.y);

            blueTeam = rbswitch.isPressed();
            delaySeconds = (delayToggle.count % 7) * 5;
            startPos = startP.count % 2;
            endPos = endP.count % 2;

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
        lblink.setRGB(0, 1, 0);
        sleep(delaySeconds * 1000);
        extendMotor.setPowerFloat();

        //This drives our robot to the beacon
        fenderDown();
        rcolor.enableLed(true);
        if(blueTeam){
           /* score = drive(0, -.35, 6800, 5, 9999);
            score = drive(45, -.35, 11000, 10, 9999) && score;
            score = drive(85, -.2, 2500, 5, 400) && score;
            drive(80, -.1, 4200, 3, 400);
            if(score) climberScore();
            if(endPos == 1) drive(85, .35, 11500, 5, 9999);*/
        }
        else {
        }


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

            telemetry.addData("Dist: ", dist.getValue());
            telemetry.addData("red", rcolor.red());
            telemetry.addData("blue", rcolor.blue());
            telemetry.addData("green", rcolor.green());

            if(colorStop && rcolor.green()>5) break;

            //The gyro sensor allows the robot to drive in a straight line regardless of debris

            throttle = Range.clip(throttle, -1, 1);
            double right = throttle + error * .02;
            double left = throttle - error * .02;
            if(throttle < 0) {
                right = Range.clip(right, Math.max(2*throttle, -1), 0);
                left = Range.clip(left, Math.max(2*throttle, -1), 0);
            }
            else{
                right = Range.clip(right, 0, Math.min(2*throttle, 1));
                left = Range.clip(left, 0, Math.min(2*throttle, 1));
            }
            if(Math.abs(throttle) > .2) {
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

            fenderl.setPosition(0.1);
            fenderr.setPosition(0.9);
    }


    public void fenderUp() {
        fenderl.setPosition(.95);
        fenderr.setPosition(.05);
    }

    public void beacon() throws InterruptedException {
        if(rcolor.red() > rcolor.blue()){
            if(blueTeam){
                panLeft(300);
                sleep(500);
                panRight(0);
            }
            else{
                panRight(-300);
                sleep(500);
                panLeft(0);
            }
        }
        else if(rcolor.blue() > rcolor.red()){
            if(blueTeam){
                panRight(-300);
                sleep(500);
                panLeft(0);
            }
            else{
                panLeft(300);
                sleep(500);
                panRight(0);
            }
        }
    }

    public void climberScore() throws InterruptedException{
        tiltUpArm(1690);
        sleep(1000);
        beacon();
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
            tiltMotor.setPower(.18);
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

    public void panRight(int t) throws InterruptedException {
        //Pan is controlled by right joystick X value

        int PID_RANGE = 50;
        double panPos = panMotor.getCurrentPosition();
        double panTarget = t;


        while(true) {
            panPos = panMotor.getCurrentPosition();
            if (panPos < panTarget) {
                break;
            }
            panMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            panMotor.setPower(-.18);
            waitOneFullHardwareCycle();
        }
        panMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        panMotor.setPower(0);
        waitOneFullHardwareCycle();

    }

    public void panLeft(int t) throws InterruptedException {
        //Pan is controlled by right joystick X value

        int PID_RANGE = 50;
        double panPos = panMotor.getCurrentPosition();
        double panTarget = t ;

        while(true) {
            panPos = panMotor.getCurrentPosition();
            if (panPos > panTarget) {
                break;
            }
            panMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            panMotor.setPower(.18);
            waitOneFullHardwareCycle();
        }
        panMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        panMotor.setPower(0);
        waitOneFullHardwareCycle();

    }


}
