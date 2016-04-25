package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by robotics on 1/5/2016.
 */
public class DualPad {
    boolean shift1;
    boolean shift2;

    boolean a;
    boolean b;
    boolean x;
    boolean y;
    boolean dpad_left;
    boolean dpad_right;
    boolean dpad_up;
    boolean dpad_down;
    boolean left_bumper;
    boolean right_bumper;
    boolean left_trigger_1_enable = false;
    boolean right_trigger_1_enable = false;
    boolean left_trigger_2_enable=false;
    boolean right_trigger_2_enable=false;
    float left_stick_x;
    float left_stick_y;
    float right_stick_x;
    float right_stick_y;
    float left_trigger=0;
    float right_trigger=0;

    boolean shift_a;
    boolean shift_b;
    boolean shift_x;
    boolean shift_y;
    boolean shift_dpad_left;
    boolean shift_dpad_right;
    boolean shift_dpad_up;
    boolean shift_dpad_down;
    boolean shift_left_bumper;
    boolean shift_right_bumper;
    float shift_left_stick_x;
    float shift_left_stick_y;
    float shift_right_stick_x;
    float shift_right_stick_y;
    float shift_left_trigger;
    float shift_right_trigger;

    public void setPads(Gamepad gamepad1, Gamepad gamepad2) {

        if (gamepad1.left_trigger >= 0.9 ){
            left_trigger_1_enable = true;
        }
        if( gamepad2.left_trigger >=.9)left_trigger_2_enable=true;

        if (gamepad1.right_trigger >=.9 ){
            right_trigger_1_enable = true;
        }
        if( gamepad2.right_trigger >= .9) right_trigger_2_enable=true;

        shift1 = gamepad1.left_bumper;
        shift2 = gamepad2.left_bumper;

        a = gamepad1.a && !shift1 || gamepad2.a && !shift2;
        shift_a = gamepad1.a && shift1 || gamepad2.a && shift2;

        b = gamepad1.b && !shift1 || gamepad2.b && !shift2;
        shift_b = gamepad1.b && shift1 || gamepad2.b && shift2;

        x = gamepad1.x && !shift1 || gamepad2.x && !shift2;
        shift_x = gamepad1.x && shift1 || gamepad2.x && shift2;

        y = gamepad1.y && !shift1 || gamepad2.y && !shift2;
        shift_y = gamepad1.y && shift1 || gamepad2.y && shift2;

        dpad_left = gamepad1.dpad_left && !shift1 || gamepad2.dpad_left && !shift2;
        shift_dpad_left = gamepad1.dpad_left && shift1 || gamepad2.dpad_left && shift2;

        dpad_down = gamepad1.dpad_down && !shift1 || gamepad2.dpad_down && !shift2;
        shift_dpad_down = gamepad1.dpad_down && shift1 || gamepad2.dpad_down && shift2;

        dpad_right = gamepad1.dpad_right && !shift1 || gamepad2.dpad_right && !shift2;
        shift_dpad_right = gamepad1.dpad_right && shift1 || gamepad2.dpad_right && shift2;

        dpad_up = gamepad1.dpad_up && !shift1 || gamepad2.dpad_up && !shift2;
        shift_dpad_up = gamepad1.dpad_up && shift1 || gamepad2.dpad_up && shift2;

        left_bumper = gamepad1.left_bumper && !shift1 || gamepad2.left_bumper && !shift2;
        shift_left_bumper = gamepad1.left_bumper && shift1 || gamepad2.left_bumper && shift2;

        right_bumper = gamepad1.right_bumper && !shift1 || gamepad2.right_bumper && !shift2;
        shift_right_bumper = gamepad1.right_bumper && shift1 || gamepad2.right_bumper && shift2;

        left_stick_x = gamepad1.left_stick_x;
        left_stick_y = gamepad1.left_stick_y;
        if(left_trigger_1_enable) left_trigger = gamepad1.left_trigger;
        right_stick_x = gamepad1.right_stick_x;
        right_stick_y = gamepad1.right_stick_y;
        if(right_trigger_1_enable) right_trigger = gamepad1.right_trigger;

        if (left_stick_x == 0 && left_stick_y == 0) {
            left_stick_x = gamepad2.left_stick_x;
            left_stick_y = gamepad2.left_stick_y;
        }

        if (right_stick_x == 0 && right_stick_y == 0) {
            right_stick_x = gamepad2.right_stick_x;
            right_stick_y = gamepad2.right_stick_y;
        }

        if (left_trigger == 0&&left_trigger_2_enable) left_trigger = gamepad2.left_trigger;
        if (right_trigger == 0&&right_trigger_2_enable) right_trigger = gamepad2.right_trigger;
    }
}

