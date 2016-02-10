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

public class OhTeleOp extends OpMode {

	DualPad gpads;
	boolean blueTeam = false;
	DcMotor tiltMotor;
	DcMotor panMotor;
	DcMotor extendMotor;

	double tiltTarget = 0;
	double panTarget = 0;

	Servo intakeServo;

	Toggle intakeToggle;

	DcMotor lf;
	DcMotor lb;
	DcMotor rf;
	DcMotor rb;


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
		lf = hardwareMap.dcMotor.get("lf");
		lb = hardwareMap.dcMotor.get("lb");
		rf = hardwareMap.dcMotor.get("rf");
		rb = hardwareMap.dcMotor.get("rb");
		rf.setDirection(DcMotor.Direction.REVERSE);
		rb.setDirection(DcMotor.Direction.REVERSE);
	}

	@Override
	public void loop() {
		gpads.setPads(gamepad1, gamepad2);
		tiltArm();
		extendArm();
		panArm();
		intake();
		drive();
		setColor();
		presets();
	}

	private void setColor(){

	}


	private void drive(){
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
		if(gpads.shift_right_trigger > .5) intakeServo.setPosition(1);
		else if(intakeToggle.onRelease(gpads.right_trigger > .5))
		{
			intakeServo.setPosition(0);
		}
		else{
			intakeServo.setPosition(0.5);
		}

	}

	public void presets(){
		if(gpads.y){
			tiltTarget = 2800;
			 panTarget = (blueTeam) ? -197 : 175;

		}
		if(gpads.shift_y){
			tiltTarget = 3165;
			panTarget = (blueTeam) ? -392: 392;
		}
		if(gpads.x){
			tiltTarget = 2930;
			panTarget = (blueTeam) ? 200 : -200;
		}
		if(gpads.b){
			//mid zipline climber
		}
		if(gpads.a){
			tiltTarget =  500;
			panTarget = 0;
		}
		if(gpads.shift_a){
			//
		}
		if(gpads.shift_x){
			blueTeam = true;
		}
		if(gpads.shift_b){
			blueTeam = false;
		}

	}

	public void tiltArm() {



		int PID_RANGE = 50;
		double tiltPos = tiltMotor.getCurrentPosition();
		tiltTarget -= gpads.right_stick_y * 5;

		telemetry.addData("stickY", gamepad1.right_stick_y);
		telemetry.addData("tiltTarget", tiltTarget);
		telemetry.addData("tiltPos", tiltPos);
		if (tiltPos < tiltTarget - PID_RANGE || tiltPos > tiltTarget + PID_RANGE) {
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
		double extendpower = 0;
		if (gpads.dpad_up) extendpower = 0.5;
		if (gpads.dpad_down) extendpower = -0.5;
		extendMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
		extendMotor.setPower(extendpower);
	}


	public void panArm() {
		if(tiltMotor.getCurrentPosition() > 1000) {
			int PID_RANGE = 100;
			double panPos = panMotor.getCurrentPosition();
			panTarget += gpads.right_stick_x * 5;

			telemetry.addData("panTarget", panTarget);
			telemetry.addData("panPos", panPos);
			if (panPos < panTarget - PID_RANGE || panPos > panTarget + PID_RANGE) {
				double panpower = Range.clip(panTarget - panPos, -0.5, 0.5);
				panMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
				panMotor.setPower(panpower);
			} else {
				panMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
				panMotor.setPower(0.6);
				panMotor.setTargetPosition((int) panTarget);
			}
		}
	}

	@Override
	public void stop() {
	}

}
