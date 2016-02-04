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
	DcMotor extendMotor;

	double tiltTarget = 0;
	double tiltStall = 0;

	public OhTeleOp() {
	}

	@Override
	public void init() {
		gpads = new DualPad();
		tiltMotor = hardwareMap.dcMotor.get("tilt");
		tiltMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
		extendMotor = hardwareMap.dcMotor.get("extend");
		extendMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
	}

	@Override
	public void loop() {
		gpads.setPads(gamepad1, gamepad2);
		tiltArm();
	}

	public void tiltArm() {
		double tiltPos = tiltMotor.getCurrentPosition();
		tiltTarget += gpads.right_stick_y * 5;
		if (gpads.a) tiltTarget = -1600;
		if (gpads.shift_a) tiltTarget = -100;

		telemetry.addData("tiltTarget", tiltTarget);
		telemetry.addData("tiltPos", tiltPos);
		telemetry.addData("tilt.isbusy", tiltMotor.isBusy());
		if (tiltPos < tiltTarget - 50 || tiltPos > tiltTarget + 50) {
			double tiltpower = Range.clip((tiltTarget - tiltPos) * 0.05, -0.2, 0.2);
			tiltMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
			tiltMotor.setPower(tiltpower);
		}
		else {
			tiltMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
			tiltMotor.setPower(0.2);
			tiltMotor.setTargetPosition((int)tiltTarget);
		}
	}

	@Override
	public void stop() {
	}

}
