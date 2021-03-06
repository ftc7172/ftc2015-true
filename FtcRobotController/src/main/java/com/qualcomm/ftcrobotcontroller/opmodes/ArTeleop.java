/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class ArTeleop extends OpMode {

    //Our gamepad wrapper class
    DualPad gpads;

    //The team we are on for the match. Defaults to Red.
    boolean blueTeam = false;
    boolean stacking = false;
    boolean ezero = true;

    DcMotor tiltMotor;
    DcMotor panMotor;
    DcMotor extendMotor;
    Servo rZip;
    Servo bZip;
    Servo intakeL;


    //The extend zero-switch
    TouchSensor armTouch;

    //Sets the team color each match
    TouchSensor rbSwitch;

   OpticalDistanceSensor opDist;

    double tiltTarget = 0;
    double panTarget = 0;


    Servo intakeR;

    Toggle intakeToggle;

    DcMotor winchMotor;
    double winchPower = 0.0;

    DcMotor lf;
    DcMotor lb;
    DcMotor rf;
    DcMotor rb;

    Toggle fenderToggle;

    Toggle zipToggle;
    AnalogInput dist;
    Boolean holdPos = false;

    Servo fenderl;
    Servo fenderr;
    ColorSensor fcolor;


    public ArTeleop() {
    }

    @Override
    public void init() {
        // initialize servos first
        rZip = hardwareMap.servo.get("rzip");
        rZip.setPosition(Arbot.RZIP_UP);
        bZip= hardwareMap.servo.get("bzip");
        bZip.setPosition(Arbot.BZIP_UP);
        intakeR = hardwareMap.servo.get("intake");
        intakeR.setPosition(.5);
        intakeL = hardwareMap.servo.get("intake1");
        intakeL.setPosition(.5);
        fenderl = hardwareMap.servo.get("lfender");
        fenderr = hardwareMap.servo.get("rfender");
        // leave fenders alone until play is pressed

        fcolor = hardwareMap.colorSensor.get("fcolor");
        dist = hardwareMap.analogInput.get("dis");
        gpads = new DualPad();
        tiltMotor = hardwareMap.dcMotor.get("tilt");
        tiltMotor.setDirection(DcMotor.Direction.REVERSE);
        tiltMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        panMotor = hardwareMap.dcMotor.get("pan");
        panMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        panMotor.setDirection(DcMotor.Direction.REVERSE);
        extendMotor = hardwareMap.dcMotor.get("extend");
        extendMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        opDist = hardwareMap.opticalDistanceSensor.get("opdist");
        opDist.enableLed(false);
        fenderToggle = new Toggle();
        intakeToggle = new Toggle();
        zipToggle = new Toggle();

        rbSwitch = hardwareMap.touchSensor.get("rbswitch");
        armTouch = hardwareMap.touchSensor.get("ezero");
        winchMotor = hardwareMap.dcMotor.get("winch");
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        if(rbSwitch.isPressed()){
            blueTeam = true;
        }

    }

    @Override
    public void loop() {
        gpads.setPads(gamepad1, gamepad2);
        presets();
        telemetry.addData("Blue?", blueTeam);
        tiltArm();
        extendArm();
        panArm();
        intake();
        drive();
        //line();
        zipTriggers();
        winch();
        fender();
        if(gpads.shift_a){
            panMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            tiltMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        }
    }

    private void line(){
        telemetry.addData("red", fcolor.red());
        telemetry.addData("blue", fcolor.blue());
        telemetry.addData("green", fcolor.green());
        telemetry.addData("alpha", fcolor.alpha());
        telemetry.addData("Dist: ", opDist.getLightDetectedRaw());
    }


    private void drive() {

        //Driving is controlled by the left joystick
        float throttle = -gpads.left_stick_y;
        if(throttle != 0){
            holdPos =false;
        }
        float direction = gpads.left_stick_x * .5f;
        double up = gpads.left_trigger * -0.5;
        if(up != 0){
            holdPos = true;
        }

        if(holdPos && gpads.left_trigger == 0){
            lf.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            rf.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            lb.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            rb.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

            int lfenc = lf.getCurrentPosition();
            int rfenc = rf.getCurrentPosition();
            int lbenc = lb.getCurrentPosition();
            int rbenc = rb.getCurrentPosition();

            lf.setTargetPosition(lfenc);
            rf.setTargetPosition(rfenc);
            lb.setTargetPosition(lbenc);
            rb.setTargetPosition(rbenc);

           lf.setPower(.35);
            rf.setPower(.35);
           lb.setPower(.35);
            rb.setPower(.35);

        }
        else {

            double right = throttle - direction;
            double left = throttle + direction;

            right = Range.clip(right, -.95, .95);
            left = Range.clip(left, -.95, .95);

            if (right == 0 && left == 0 && up < 0) {
                lf.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                lb.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                rf.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                rb.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                right = up;
                left = up;
            } else {
                lf.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                lb.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                rf.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                rb.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            }

            /*if(rf.getPower() != 0 || lf.getPower() != 0 || lb.getPower() != 0 || rb.getPower() != 0 ||right != 0 || left!= 0)
            {
                if(rf.getPower() < right)
                {
                    rf.setPower(rf.getPower() + 0.06);
                }
                else if(rf.getPower() >= right)
                 {
                    rf.setPower(rf.getPower() - 0.06);
                 }
                else rf.setPower(right);

                //if(lf.getPower() == 0) return;
                if(lf.getPower() < left)
                {
                    lf.setPower(lf.getPower() + 0.06);
                }
                else if(lf.getPower() > left)
                {
                    lf.setPower(lf.getPower() - 0.06);
                }
                else lf.setPower(left);

                if(lb.getPower() < left)
                {
                    lb.setPower(lb.getPower() + 0.06);
                }
                else if(lb.getPower() >= left)
                {
                    lb.setPower(lb.getPower() - 0.06);
                }
                else lb.setPower(left);

                if(rb.getPower() < right)
                {
                    rb.setPower(rb.getPower() + 0.06);
                }
                else if(rb.getPower() >= right)
                {
                    rb.setPower(rb.getPower() - 0.06);
                }
                else rb.setPower(right);
            }*/
           /* else {*/
                rf.setPower(right);
                rb.setPower(right);
                lf.setPower(left);
                lb.setPower(left);
           // }*/

        }

    }

    public void intake(){

        //The intake is toggled by the  trigger
        double intakePosL = 0.5;
        double intakePosR = 0.5;
        if(gpads.dpad_right){
            intakePosL = 0;
            intakePosR = 1;
        }
        if(tiltMotor.getCurrentPosition() < 1600){
            if(intakeToggle.onRelease(gpads.right_trigger > .5)){
                intakePosL = 1;
                intakePosR = 0;
            }
        }
        else{
            if(gpads.right_trigger > .5){
                intakePosL = 0;
                intakePosR = 1;
            }

        }
        /*
        if(intakeToggle.onRelease(gpads.right_trigger > .5))
        {
            //If the arm is in scoring position, run the intake in reverse
            intakePosL = (tiltMotor.getCurrentPosition() < 1600) ? 1: 0;
            intakePosR = (tiltMotor.getCurrentPosition() < 1600) ? 0 : 1;
        }*/
        if(gpads.right_bumper){
            intakePosL = (tiltMotor.getCurrentPosition() < 1600) ? 0: 1;
            intakePosR = (tiltMotor.getCurrentPosition() < 1600) ? 1 : 0;
        }
        intakeR.setPosition(intakePosR);
        intakeL.setPosition(intakePosL);

    }

    public void zipTriggers(){
        if(zipToggle.onRelease(gpads.shift_dpad_left)){
            if(blueTeam){
                bZip.setPosition(Arbot.BZIP_DOWN);
            }
            else{
                rZip.setPosition(Arbot.RZIP_DOWN);
            }
        }
        else {
            bZip.setPosition(Arbot.BZIP_UP);
            rZip.setPosition(Arbot.RZIP_UP);
        }
    }

    public void presets(){
        //The Y button extends the arm to high basket scoring position
        if(gpads.y){
            tiltTarget = 2650;
            panTarget = (blueTeam) ? -250 : 173;
            intakeToggle.onoff = false;

        }
        //Shift-Y extends the arm to mid basket scoring position
        if(gpads.shift_y){
            tiltTarget = 2925;
            panTarget = (blueTeam) ? -412: 412;
            intakeToggle.onoff = false;
        }
        //The X button extends the arm to High Zipline Climber position
        if(gpads.x){
            tiltTarget = 2880;
            panTarget = (blueTeam) ? 200 : -200;
        }
        if(gpads.dpad_right){
          //  tiltTarget = tiltTarget + 60;
            //while(tiltMotor.getCurrentPosition() < tiltTarget) {
                intakeL.setPosition(.3);
                intakeR.setPosition(.7);
            //    extendMotor.setPower(-.5);

        }
        //The A button returns the arm to driving position
        if(gpads.a){
            tiltTarget =  300;
            panTarget = 0;
            intakeToggle.onoff = false;
        }
        if(gpads.shift_a){
            //Sets 0 point
        }
        if(gpads.shift_dpad_up){
            //High Zone Parking
            tiltTarget = 2600;
            panTarget = 0;
        }
        //Shift X and B (Blue and Red buttons) serve to change the team if the robot switch fails
        if(gpads.shift_x){
            blueTeam = true;
        }
        if(gpads.shift_b){
            blueTeam = false;
        }

    }

    public void tiltArm() {
        //Arm tilt is controlled by the right joystick's Y value
        int PID_RANGE = 50;
        double tiltPos = tiltMotor.getCurrentPosition();
        tiltTarget -= gpads.right_stick_y * 5;

        telemetry.addData("stickY", gamepad1.right_stick_y);
        telemetry.addData("tiltTarget", tiltTarget);
        telemetry.addData("tiltPos", tiltPos);

        if (intakeToggle.isOnoff()&& tiltMotor.getCurrentPosition()<500){
            tiltMotor.setPowerFloat();
            tiltTarget = tiltMotor.getCurrentPosition();
        }
        else if(gpads.b){
            tiltMotor.setPowerFloat();
            tiltTarget = tiltMotor.getCurrentPosition();
        }
        //This statement prevents the arm from surpassing a certain speed while tilting
        //which protects the robot in case of PID failure
        else if (tiltPos < tiltTarget - PID_RANGE || tiltPos > tiltTarget + PID_RANGE) {
            double tiltpower = Range.clip(tiltTarget - tiltPos, -0.3, 0.3);
            tiltMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            tiltMotor.setPower(tiltpower);
        }
        else {
            tiltMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            tiltMotor.setPower(0.2);
            tiltMotor.setTargetPosition((int) tiltTarget);
        }
    }

    public void extendArm()
    {
        //Extension is controlled by up and down on the Dpad
        double extendpower = 0;
        extendMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        if(gpads.shift_dpad_down){
            extendpower = -1;
            ezero = false;
        }
        if(ezero && armTouch.isPressed()){

            extendMotor.setPower(.5);
            return;
        }
        if (gpads.dpad_up) {extendpower = 1; ezero = true;}
        if (gpads.dpad_down) {extendpower = -1; ezero = true;}

        extendMotor.setPower(extendpower);
        telemetry.addData("Ezero", armTouch.isPressed());
    }


    public void panArm() {
        //Pan is controlled by right joystick X value

        int PID_RANGE = 50;
        double panPos = panMotor.getCurrentPosition();
        panTarget += gpads.right_stick_x * -2;

        telemetry.addData("panTarget", panTarget);
        telemetry.addData("panPos", panPos);
        if (tiltMotor.getCurrentPosition()>1000 || gpads.right_stick_x != 0) {
            //This statement prevents the arm from surpassing a certain speed while panning
            //which protects the robot in case of PID failure

            if (panPos < panTarget - PID_RANGE || panPos > panTarget + PID_RANGE) {
                double panpower = Range.clip(panTarget - panPos, -0.2, 0.2);
                panMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                panMotor.setPower(panpower);
            } else {
                panMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                panMotor.setPower(0.2);
                panMotor.setTargetPosition((int) panTarget);
            }
        }
    }
    public void winch() {
        winchMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        if (gamepad1.back) {
            winchPower = 1;
        }
        else {
            winchPower=0;
        }
        winchMotor.setPower(winchPower);
    }
    public void fender() {
        if (fenderToggle.onRelease(gpads.shift_dpad_right)){
            fenderl.setPosition(Arbot.LFENDER_DOWN);
            fenderr.setPosition(Arbot.RFENDER_DOWN);
        }
        else {
            fenderl.setPosition(Arbot.LFENDER_UP);
            fenderr.setPosition(Arbot.RFENDER_UP);
        }
    }


    @Override
    public void stop() {
    }

}
