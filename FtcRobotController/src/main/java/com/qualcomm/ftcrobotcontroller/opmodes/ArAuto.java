
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

/**
 * Created by pmichaud on 2/8/2016.
 */
public class ArAuto extends LinearOpMode {
    DcMotor tiltMotor;
    // DcMotor panMotor;

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

    TouchSensor rbswitch;
    Toggle startP;
    Toggle endP;
    boolean blueTeam;
    int startPos;
    int endPos;


    Toggle delayToggle;
    int delaySeconds;


    BlinkM lblink;
    OpticalDistanceSensor opD;


    public ArAuto() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        dist = hardwareMap.analogInput.get("dis");
        tiltMotor = hardwareMap.dcMotor.get("tilt");
        tiltMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        tiltMotor.setDirection(DcMotor.Direction.REVERSE);
        //  panMotor = hardwareMap.dcMotor.get("pan");
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

        gpads = new DualPad();
        rbswitch = hardwareMap.touchSensor.get("rbswitch");
        delayToggle = new Toggle();
        startP = new Toggle();
        endP = new Toggle();
        rZip = hardwareMap.servo.get("rzip");
        bZip = hardwareMap.servo.get("bzip");
        opD = hardwareMap.opticalDistanceSensor.get("opdist");
        opD.enableLed(false);
        intakeServo = hardwareMap.servo.get("intake");
        intakeL= hardwareMap.servo.get("intake1");
        bZip.setPosition(.12);
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

        if(blueTeam){
            score = drive(0, -.35, 6800, 5, 9999);
            score = drive(45, -.35, 11000, 10, 9999) && score;
            score = drive(85, -.2, 2500, 5, 400) && score;
            drive(80, -.1, 4200, 3, 400);
            if(score) climberScore();
            if(endPos == 1) drive(85, .35, 11500, 5, 9999);
        }
        else {
            score = drive(0, -.35, 6800, 5, 9999);
            score = drive(-45, -.35, 10000, 10, 9999) && score;
            score = drive(-80, -.2, 2500, 5, 400) && score;
            drive(-85, -.1, 4200, 3, 400);
           // score = drive(0, -.35, 2500, 10, 9999);
          //  score = drive(-30, -.35, 11000, 10, 9999);
            if(score) climberScore();
            if(endPos == 1) drive(-85, .35, 11500, 5, 9999);
        }

        //This positions the arm to score the climbers



    }

    public boolean drive(double targetAngle, double throttle, double distance, double timeout, double stopd)throws InterruptedException{
        int pastRPos = rb.getCurrentPosition();
        int pastLPos = lb.getCurrentPosition();
        double startTime = time;
        double traveled = distance;
        while (time < startTime + timeout){
            double heading = gyro.getHeading() > 180 ? gyro.getHeading() - 360: gyro.getHeading();
            double error = (heading) - targetAngle;

           // if(Math.abs(error) < 2){
                traveled -= Math.cos(error * Math.PI/180)*(Math.abs(rb.getCurrentPosition() - pastRPos) + Math.abs(lb.getCurrentPosition()-pastLPos));
           // }

            if (traveled<=0 ){
                    break;
                }
            pastRPos = rb.getCurrentPosition();
            pastLPos = lb.getCurrentPosition();
            //lblink.setRGB(0, 1, 0);

            if(dist.getValue() > stopd) break;
            telemetry.addData("Dist: ", dist.getValue());

           // telemetry.addData("opD", opD.getLightDetectedRaw());
            //if(opD.getLightDetectedRaw() > 10){
             //   break;
          //  }
           // telemetry.addData("lf", lf.getCurrentPosition() - pastLPos);
            //The gyro sensor allows the robot to drive in a straight line regardless of debris

            throttle = Range.clip(throttle, -1, 1);
            double right = throttle + error * .02;
            double left = throttle - error * .02;
            if(throttle < 0) {
                right = Range.clip(right, -1, 0);
                left = Range.clip(left, -1, 0);
            }
            else{
                right = Range.clip(right, -1, 1);
                left = Range.clip(left, -1, 1);
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
        tiltUpArm(1690);
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


}
