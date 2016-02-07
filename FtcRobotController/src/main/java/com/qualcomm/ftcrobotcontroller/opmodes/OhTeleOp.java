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
		intakeServo = hardwareMap.servo.get("intake");
		tiltMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
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
		//extendArm();
		//panArm();
		intake();
		drive();
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
		if(gpads.shift_x) intakeServo.setPosition(0);
		else if(intakeToggle.onRelease(gpads.x))
		{
			intakeServo.setPosition(1);
		}
		else{
			intakeServo.setPosition(0.5);
		}

	}

	public void tiltArm() {
		int PID_RANGE = 50;
		double tiltPos = tiltMotor.getCurrentPosition();
		tiltTarget += gpads.right_stick_y * 5;
		if (gpads.a) tiltTarget = -1600;
		if (gpads.shift_a) tiltTarget = -100;

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
			tiltMotor.setTargetPosition((int)tiltTarget);
		}
	}

	public void extendArm()
	{
		double extendRaw = extendMotor.getCurrentPosition();
		double extendCmd = gamepad1.right_stick_x;

		if(extendCmd != 0){
			extendMotor.setTargetPosition(extendMotor.getCurrentPosition()+10);
			return;
		}
		else{
			extendMotor.setTargetPosition(extendMotor.getCurrentPosition());

		}

		telemetry.addData("extend",extendMotor.getCurrentPosition());
	}
	public void panArm() {
		int PID_RANGE = 50;
		double panPos = panMotor.getCurrentPosition();
		panTarget += gpads.right_stick_x * 5;

		telemetry.addData("panTarget", panTarget);
		telemetry.addData("panPos", panPos);
		if (panPos < panTarget - PID_RANGE || panPos > tiltTarget + PID_RANGE) {
			double panpower = Range.clip(panTarget - panPos, -0.2, 0.2);
			panMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
			panMotor.setPower(panpower);
		}
		else {
			panMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
			panMotor.setPower(0.2);
			panMotor.setTargetPosition((int)panTarget);
		}
	}

	@Override
	public void stop() {
	}

}
