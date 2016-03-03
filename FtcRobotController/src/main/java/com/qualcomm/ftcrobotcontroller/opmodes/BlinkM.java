package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by robotics on 1/5/2016.
 */
public class BlinkM {
    I2cDevice blinkm;
    int i2caddr = 0x12;
    byte cmd[] = { 0, 0, 0, 0, 0 };

    BlinkM(I2cDevice dev) {
        blinkm = dev;
    }

    public boolean isReady() {
        return blinkm.isI2cPortReady();
    }

    public void setRGB(double r, double g, double b) {
        cmd[0] = (byte)(Range.clip(r, 0, 1) * 255);
        cmd[1] = (byte)(Range.clip(g, 0, 1) * 255);
        cmd[2] = (byte)(Range.clip(b, 0, 1) * 255);
        blinkm.enableI2cWriteMode(i2caddr, 0x6e, 3);
        blinkm.copyBufferIntoWriteBuffer(cmd);
        blinkm.setI2cPortActionFlag();
        blinkm.writeI2cCacheToController();
     }

    public void runProgram(int n) {
        cmd[0] = (byte)n;
        cmd[1] = 0;
        cmd[2] = 0;
        blinkm.enableI2cWriteMode(i2caddr, 0x70, 3);
        blinkm.copyBufferIntoWriteBuffer(cmd);
        blinkm.setI2cPortActionFlag();
        blinkm.writeI2cCacheToController();
    }

}

