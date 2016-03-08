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

    Servo fenderl;
    Servo fenderr;
    ColorSensor fcolor;


    public ArTeleop() {
    }

    @Override
    public void init() {
        fcolor = hardwareMap.colorSensor.get("fcolor");
        dist = hardwareMap.analogInput.get("dis");
        gpads = new DualPad();
        tiltMotor = hardwareMap.dcMotor.get("tilt");
        tiltMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeR = hardwareMap.servo.get("intake");
        intakeL = hardwareMap.servo.get("intake1");
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
        rZip = hardwareMap.servo.get("rzip");
        rZip.setPosition(.85);
        bZip= hardwareMap.servo.get("bzip");
        bZip.setPosition(.12);
        intakeR.setPosition(.5);
        intakeL.setPosition(.5);
        fenderl = hardwareMap.servo.get("lfender");
        fenderr = hardwareMap.servo.get("rfender");

        fenderl.setPosition(fenderl.getPosition());
        fenderr.setPosition(fenderr.getPosition());

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
        line();
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
        float direction = gpads.left_stick_x * .5f;
        double up = gpads.left_trigger * -0.5;

        double right = throttle - direction;
        double left = throttle + direction;

        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        if (right == 0 && left == 0 && up<0){
            lf.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            lb.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            rf.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            rb.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            right = up;
            left = up;
        }
        else {
            lf.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            lb.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            rf.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            rb.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        }

        rf.setPower(right);
        rb.setPower(right);
        lf.setPower(left);
        lb.setPower(left);

    }

    public void intake(){

        //The intake is toggled by the  trigger
        double intakePosL = 0.5;
        double intakePosR = 0.5;
        if(intakeToggle.onRelease(gpads.right_trigger > .5))
        {
            //If the arm is in scoring position, run the intake in reverse
            intakePosL = (tiltMotor.getCurrentPosition() < 1600) ? 1: 0;
            intakePosR = (tiltMotor.getCurrentPosition() < 1600) ? 0 : 1;
        }
        if(gpads.right_bumper){
            intakePosL = 0;
            intakePosR = 1;
        }
        intakeR.setPosition(intakePosR);
        intakeL.setPosition(intakePosL);

    }

    public void zipTriggers(){
        if(zipToggle.onRelease(gpads.dpad_left)){
            if(blueTeam){
                bZip.setPosition(.85);
            }
            else{
                rZip.setPosition(.25);
            }
        }
        else {
            bZip.setPosition(.3);
            rZip.setPosition(0.85);
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
        if(gpads.b){
            //Middle Zipline Climber

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
            double tiltpower = Range.clip(tiltTarget - tiltPos, -0.2, 0.2);
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
        extendMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        if(armTouch.isPressed()){

            extendMotor.setPower(0.5);
            return;
        }
        double extendpower = 0;
        if (gpads.dpad_up) extendpower = 0.5;
        if (gpads.dpad_down) extendpower = -0.5;

        extendMotor.setPower(extendpower);
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
            winchPower = 0.9;
        }
        else {
            winchPower -= Range.clip(winchPower - 0.01, 0, 1);
        }
        winchMotor.setPower(winchPower);
    }
    public void fender() {
        if (fenderToggle.onRelease(gpads.shift_dpad_right)){
            fenderl.setPosition(0.1);
            fenderr.setPosition(0.9);
        }
        else {
            fenderl.setPosition(1);
            fenderr.setPosition(0);
        }
    }


    @Override
    public void stop() {
    }

}
