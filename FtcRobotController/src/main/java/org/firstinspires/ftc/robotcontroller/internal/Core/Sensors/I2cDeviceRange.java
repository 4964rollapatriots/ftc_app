package org.firstinspires.ftc.robotcontroller.internal.Core.Sensors;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.HardwareMapper;

public class I2cDeviceRange {

    private RobotBase base;


    private I2cDevice range;
    private byte[] rangeCache;
    private I2cDeviceSynch reader;

    public void init(final RobotBase robot, final String NAME, I2cAddr Address){

        HardwareMapper mapHelper = new HardwareMapper(robot);

        range = mapHelper.mapI2cDevice(NAME);

        reader = new I2cDeviceSynchImpl(range, Address, false);
        reader.engage();

    }

    public double getUltrasonicDistance(){

        rangeCache = reader.read(0x04,2);

        return rangeCache[0] & 0xFF;
    }

    public double getOpticalDistance(){

        rangeCache = reader.read(0x04, 2);
        return rangeCache[1] & 0xFF;
    }
}
