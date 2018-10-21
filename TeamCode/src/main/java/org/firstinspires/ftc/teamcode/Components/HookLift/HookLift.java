package org.firstinspires.ftc.teamcode.Components.HookLift;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.RobotComponent;

/**
 * Created by pmkf2 on 10/17/2018.
 */

public class HookLift extends RobotComponent
{
    private DcMotor raiseRobotSpool;
    private Servo extendHookLift;

    public void init(RobotBase BASE)
    {
        raiseRobotSpool = mapper.mapMotor("spool", DcMotorSimple.Direction.FORWARD);
        raiseRobotSpool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //change this later,once the amount of encoder
        //counts needed to lift up are found.
        extendHookLift = mapper.mapServo("lift", Servo.Direction.FORWARD);
    }

    public void extendLift(int POSITION)
    {
        extendHookLift.setPosition(POSITION);
    }
    public void retractLift()
    {
        extendHookLift.setPosition(0);
    }
    public void liftRobot(int POWER)
    {
        raiseRobotSpool.setPower(POWER);
    }

    public void liftRobot(int POWER, int DIST)
    {
        raiseRobotSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        raiseRobotSpool.setTargetPosition(DIST);
        do {
            raiseRobotSpool.setPower(POWER);
        }while(!(raiseRobotSpool.getCurrentPosition() >= raiseRobotSpool.getTargetPosition() - 7 ||
                raiseRobotSpool.getCurrentPosition() <=  raiseRobotSpool.getTargetPosition() + 7));
        raiseRobotSpool.setPower(0);
    }

    public void lowerRobot()
    {
        raiseRobotSpool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //later change this to go to position 0 on encoder counts
        raiseRobotSpool.setPower(-.75);
    }

    @Override
    public void stop()
    {
        raiseRobotSpool.setPower(0);
    }
}
