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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class OhTeleOp extends OpMode {

	//Our gamepad wrapper class
	DualPad gpads;

	//The team we are on for the match. Defaults to Red.
	boolean blueTeam = false;

	DcMotor tiltMotor;
	DcMotor panMotor;
	DcMotor extendMotor;
	Servo rZip;
	Servo bZip;

	//The extend zero-switch
	TouchSensor armTouch;

	//Sets the team color each match
	TouchSensor rbSwitch;

	double tiltTarget = 0;
	double panTarget = 0;

	Servo intakeServo;

	Toggle intakeToggle;

	DcMotor lf;
	DcMotor lb;
	DcMotor rf;
	DcMotor rb;

	Toggle zipToggle;


	public OhTeleOp() {
	}

	@Override
	public void init() {
		gpads = new DualPad();
		tiltMotor = hardwareMap.dcMotor.get("tilt");
		tiltMotor.setDirection(DcMotor.Direction.REVERSE);
		intakeServo = hardwareMap.servo.get("intake");
		tiltMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		panMotor = hardwareMap.dcMotor.get("pan");
		panMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		panMotor.setDirection(DcMotor.Direction.REVERSE);
		extendMotor = hardwareMap.dcMotor.get("extend");
		extendMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		intakeToggle = new Toggle();
		zipToggle = new Toggle();
		rbSwitch = hardwareMap.touchSensor.get("rbswitch");
		armTouch = hardwareMap.touchSensor.get("ezero");
		lf = hardwareMap.dcMotor.get("lf");
		lb = hardwareMap.dcMotor.get("lb");
		rf = hardwareMap.dcMotor.get("rf");
		rb = hardwareMap.dcMotor.get("rb");
		rf.setDirection(DcMotor.Direction.REVERSE);
		rb.setDirection(DcMotor.Direction.REVERSE);
		rZip = hardwareMap.servo.get("rzip");
		bZip= hardwareMap.servo.get("bzip");
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
		zipTriggers();
		if(gpads.shift_a){
			panMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
			tiltMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		}
	}


	private void drive(){
		//Driving is controlled by the left joystick
		float throttle = -gpads.left_stick_y;
		float direction = gpads.left_stick_x;

		float right = throttle - direction;
		float left = throttle + direction;

		right = Range.clip(right, -1, 1);
		left = Range.clip(left, -1, 1);

		rf.setPower(right);
		rb.setPower(right);
		lf.setPower(left);
		lb.setPower(left);


	}

	public void intake(){

		//The intake is toggled by the  trigger
		double intakePos = 0.5;
		if(intakeToggle.onRelease(gpads.right_trigger > .5))
		{
			//If the arm is in scoring position, run the intake in reverse
			intakePos = (tiltMotor.getCurrentPosition() < 1600) ? 0 : 1;
		}
		if(gpads.right_bumper){
			intakePos = 1;
		}
		intakeServo.setPosition(intakePos);
	}

	public void zipTriggers(){
		if(zipToggle.onRelease(gpads.dpad_left)){
			if(blueTeam){
				bZip.setPosition(.9);
			}
			else{
				rZip.setPosition(.1);
			}
		}
		else {
			bZip.setPosition(0);
			rZip.setPosition(1);
		}
	}

	public void presets(){
		//The Y button extends the arm to high basket scoring position
		if(gpads.y){
			tiltTarget = 2800;
			 panTarget = (blueTeam) ? -197 : 175;
			intakeToggle.onoff = false;

		}
		//Shift-Y extends the arm to mid basket scoring position
		if(gpads.shift_y){
			tiltTarget = 3000;
			panTarget = (blueTeam) ? -392: 392;
			intakeToggle.onoff = false;
		}
		//The X button extends the arm to High Zipline Climber position
		if(gpads.x){
			tiltTarget = 2830;
			panTarget = (blueTeam) ? 200 : -200;
		}
		if(gpads.b){
			//Middle Zipline Climber
			tiltTarget = 3120;
			panTarget = (blueTeam) ? 490: -490;
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
		if(armTouch.isPressed()){
			extendMotor.setPower(0.5);
			return;
		}
		double extendpower = 0;
		if (gpads.dpad_up) extendpower = 0.5;
		if (gpads.dpad_down) extendpower = -0.5;
		extendMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
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

	@Override
	public void stop() {
	}

}
