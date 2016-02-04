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
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class EnTeleOp extends OpMode {

	DualPad gpads;

	DcMotor tiltMotor;
	DcMotor extendmotor;
	DcMotor intakeMotor;

	DcMotor lf;
	DcMotor lb;
	DcMotor rf;
	DcMotor rb;

	Servo hopperServo;
	Servo beltServo;
	Servo drawerServo;
	Servo latchServo;

	OpticalDistanceSensor ods;

	Toggle intakeToggle;
	Toggle upToggle;
	boolean abuttonHit = false;

	boolean blueTeam;

	double extendTarget;

	public EnTeleOp() {

	}

	@Override
	public void init() {
		drawerServo = hardwareMap.servo.get("drawer");
		drawerServo.setPosition(0.5);

		beltServo = hardwareMap.servo.get("belt");
		beltServo.setPosition(0.5);

		hopperServo = hardwareMap.servo.get("hopper");
		hopperServo.setPosition(0.5);

		latchServo = hardwareMap.servo.get("test");
		latchServo.setPosition(0.5);

		gpads = new DualPad();

		lf = hardwareMap.dcMotor.get("lf");
		lb = hardwareMap.dcMotor.get("lb");
		rf = hardwareMap.dcMotor.get("rf");
		rb = hardwareMap.dcMotor.get("rb");
		rf.setDirection(DcMotor.Direction.REVERSE);
		rb.setDirection(DcMotor.Direction.REVERSE);

		intakeMotor = hardwareMap.dcMotor.get("intake");
		intakeMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
		intakeMotor.setDirection(DcMotor.Direction.REVERSE);
		ods = hardwareMap.opticalDistanceSensor.get("ods");
		ods.enableLed(true);
		intakeToggle = new Toggle();
		upToggle = new Toggle();

		extendmotor = hardwareMap.dcMotor.get("extend");

		tiltMotor = hardwareMap.dcMotor.get("tilt");


		blueTeam = true;


	}


	@Override
	public void loop() {
		gpads.setPads(gamepad1, gamepad2);
		drive();
		sweepBlocks();
		setTeam();
		tiltarm();
		extendArm();
		hopper();
		beltdrawer();
		drawer();
		latch();

	}

	private void drive(){
		float throttle = -gpads.left_stick_y;
		float direction = gpads.left_stick_x;

		float right = throttle - direction;
		float left = throttle + direction;

		right = Range.clip(right, -1, 1);
		left = Range.clip(left, -1, 1);

		if(right==0 && left == 0 && upToggle.onRelease(gpads.a))
		{
			right = (float)-0.6;
			left = (float)-0.6;
		}
		rf.setPower(right);
		rb.setPower(right);
		lf.setPower(left);
		lb.setPower(left);


	}

	private void tiltarm() {
		double tiltpower = 0;
		if (gpads.dpad_up) { tiltpower = 1; }
		else if (gpads.dpad_down) { tiltpower = -0.5; }
		tiltMotor.setPower(tiltpower);
	}


	//Tilting the right joystick left or right extends and retracts the arm.

	/* private void extendArm(){
		extendmotor.setPower(gamepad1.right_stick_x*.8);
		telemetry.addData("extend",extendmotor.getCurrentPosition());
		telemetry.addData("ExtendZ",extendZ.getValue());

	}*/

	//Tilting the right joystick forward or backward tilts the arm
	 private void extendArm(){
		double extendRaw = extendmotor.getCurrentPosition();
		double extendCmd = gpads.right_stick_y;

		if(extendCmd != 0){
			extendmotor.setPower(extendCmd * 0.8);
			return;
		}
		extendmotor.setPower(0);

		 telemetry.addData("extend",extendmotor.getCurrentPosition());
	 }



	//A button toggles sweeper. Shift-A sweeps in reverse.
	private void sweepBlocks(){

		if (intakeToggle.onRelease(gpads.right_trigger > 0.5)){
			abuttonHit = true;
			if(gpads.left_bumper){
				intakeMotor.setPower(-.6);
			}
			else{
				intakeMotor.setPower(.6);
			}
		}
		else if(ods.getLightDetected() > .5 && abuttonHit){
			intakeMotor.setPower(.15);
		}
		else{
			intakeMotor.setPower(0);
		}

	}

	//Shift-X sets team to blue. Shift-B sets team to red.
	private void setTeam(){
		if (gpads.shift_x) blueTeam = true;
		if (gpads.shift_b) blueTeam = false;
		telemetry.addData("team", blueTeam ? "blue" : "red");
	}

	private void hopper(){
		if (gpads.right_bumper){
			hopperServo.setPosition(0.2);
		}
		else if (intakeToggle.isOnoff()) {
			hopperServo.setPosition(0.902);
		}
		else{
			hopperServo.setPosition(0.902);
		}
	}
	private void beltdrawer(){
		if (gpads.dpad_left){
			beltServo.setPosition(1);
		}
		else if (gpads.dpad_right){
			beltServo.setPosition(0);
		}
		else {
			beltServo.setPosition(0.5);
		}
	}
	private void drawer(){
		if (gpads.x){
			drawerServo.setPosition(1);
		}
		else if (gpads.b) {
			drawerServo.setPosition(0);
		}
		else {
			drawerServo.setPosition(0.5);
		}
	}
	private void latch(){
		if (gpads.y){
			latchServo.setPosition(1);
		}
		else {
			latchServo.setPosition(0.5);
		}
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}


}


