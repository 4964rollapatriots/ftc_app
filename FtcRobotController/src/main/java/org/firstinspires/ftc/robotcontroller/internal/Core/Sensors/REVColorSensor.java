package org.firstinspires.ftc.robotcontroller.internal.Core.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.Color.ColorID;
import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.HardwareMapper;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by pmkf2 on 9/1/2018.
 */

public class REVColorSensor
{
    private RobotBase robot;


    private DistanceSensor distance;

    //Constructor to create the color sensor

    public REVColorSensor(RobotBase myRobot)
    {
        robot = myRobot;
    }

    //Map the color sensor.

    public void mapREVDistanceSensor(final String NAME)
    {
        HardwareMapper mapHelper = new HardwareMapper(robot);

        distance = mapHelper.mapDistanceSensor(NAME);

    }



    public double distance(){
        return (distance.getDistance(DistanceUnit.INCH));
    }
}
