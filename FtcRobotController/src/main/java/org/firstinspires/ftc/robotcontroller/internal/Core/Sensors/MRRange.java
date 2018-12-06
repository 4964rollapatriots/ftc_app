package org.firstinspires.ftc.robotcontroller.internal.Core.Sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.HardwareMapper;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by pmkf2 on 12/5/2018.
 */

public final class MRRange
{
    private RobotBase robot = null; //our robot

    private ModernRoboticsI2cRangeSensor range = null; //the actual sensor


    //constructor
    MRRange(RobotBase MYROBOT)
    {
        robot = MYROBOT;
    }

    void mapRange(final String NAME)
    {
        HardwareMapper mapHelper = new HardwareMapper(robot);
        range = mapHelper.mapMRRange(NAME);
    }

    double distance (final DistanceUnit UNIT)
    {
        //range reads zero when out of range, this isn't helpful for logic though, so changing that to 1_024;

        final double DEFAULT_DIST = 1_024;

        double distance = 0; //distance read by range sensor

        distance = range.getDistance(UNIT);

        if(distance == 0 )
            distance = DEFAULT_DIST;

        return distance;
    }





}
