package org.firstinspires.ftc.robotcontroller.internal.Core.Sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.HardwareMapper;

/**
 * Created by pmkf2 on 1/6/2019.
 */

public class Touch {

    private RobotBase robot = null; //our robot

    private RevTouchSensor touch = null; //the actual sensor



    public void init(final RobotBase r, final String NAME){

        HardwareMapper mapHelper = new HardwareMapper(robot);
        this.robot = r;
        touch = mapHelper.mapTouch(NAME);
    }

    public boolean isPressed(){
        boolean result = false;
        if (touch != null){
            try{
                result = touch.isPressed();
            }
            catch(Exception e){
                robot.outTelemetry.write("Touch sensor unplugged");
            }
        }

        return result;
    }
}
