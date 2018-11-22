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
    private DcMotor winch;
    private Servo extendHookLift;

    private int LIFT_TO_POS = 50;

    public void init(RobotBase BASE)
    {
        winch = mapper.mapMotor("winch", DcMotorSimple.Direction.FORWARD);
        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //change this later,once the amount of encoder
        //counts needed to lift up are found.
        extendHookLift = mapper.mapServo("lift", Servo.Direction.FORWARD);
    }

    public void extendLift()
    {
        extendHookLift.setPosition(extendHookLift.getPosition() + .05);
    }
    public void extendToPos()
    {
        extendHookLift.setPosition(LIFT_TO_POS);
    }
    public void retractLift()
    {
        extendHookLift.setPosition(extendHookLift.getPosition() - .05);
    }
    public void liftRobot(double POWER)
    {
        winch.setPower(POWER);
    }

    public void liftRobot(int POWER, int DIST)
    {
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winch.setTargetPosition(DIST);
        do {
            winch.setPower(POWER);
        }while(!(winch.getCurrentPosition() >= winch.getTargetPosition() - 7 &&
                winch.getCurrentPosition() <=  winch.getTargetPosition() + 7));
        winch.setPower(0);
    }

    public void lowerRobot()
    {
        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //later change this to go to position 0 on encoder counts
        winch.setPower(-.30);
    }

    public void outTelemetry()
    {
        base.outTelemetry.write("----------------HOOK LIFT---------------");
        base.outTelemetry.addData("Winch Power", winch.getPower());
        base.outTelemetry.addData("Winch Position", winch.getCurrentPosition());
        base.outTelemetry.addData("ExtendHook Position", extendHookLift.getPosition());
        base.outTelemetry.write("-------------END HOOK LIFT ---------");
        base.outTelemetry.newLine();
    }

    @Override
    public void stop()
    {
        winch.setPower(0);
    }
}
