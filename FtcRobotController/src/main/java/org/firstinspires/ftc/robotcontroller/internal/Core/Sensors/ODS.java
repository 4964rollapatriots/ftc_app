package org.firstinspires.ftc.robotcontroller.internal.Core.Sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.HardwareMapper;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by pmkf2 on 12/5/2018.
 */

public class ODS
{
    private RobotBase robot = null; //our robot

    private OpticalDistanceSensor sensor = null;



    public void init(final RobotBase robot, final String NAME){

        HardwareMapper mapHelper = new HardwareMapper(robot);

        sensor = mapHelper.mapODS(NAME);
    }


    public double raw ()
    {
        double answer = -1;
        try{
            answer = sensor.getRawLightDetected();
        }
        catch(Exception e){e.printStackTrace();}

        return answer;
    }

    public double normal ()
    {
        double answer = -1;
        try{
            answer = sensor.getLightDetected();
        }
        catch(Exception e){e.printStackTrace();}

        return answer;
    }





}
