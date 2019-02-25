package org.firstinspires.ftc.robotcontroller.internal.Core.Sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.HardwareMapper;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;

/**
 * Created by pmkf2 on 12/5/2018.
 */

public final class REVMagnetic
{
    private RobotBase robot = null; //our robot

    private DigitalChannel limitSwitch = null; //the actual sensor



    public void init(final RobotBase robot, final String NAME){

        HardwareMapper mapHelper = new HardwareMapper(robot);

        limitSwitch = mapHelper.mapDigitalChannel(NAME);
    }


    public boolean isClose ()
    {
        return !this.limitSwitch.getState();
    }





}
