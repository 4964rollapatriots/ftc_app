package org.firstinspires.ftc.teamcode.Components.HookLift;

import com.qualcomm.robotcore.hardware.CRServo;
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
    private CRServo extendHookLift;

    private int LIFT_TO_POS = 50;
    private double SERVO_POWER = .40;

    public void init(RobotBase BASE)
    {
        winch = mapper.mapMotor("winch");
        if (winch == null){
            base.outTelemetry.write("winch is null");
            base.outTelemetry.update();
        }
        //counts needed to lift up are found.
        extendHookLift = mapper.mapCRServo("lift", CRServo.Direction.FORWARD);
    }

    public void extendHook()
    {
        extendHookLift.setPower(SERVO_POWER);

    }
    //public void extendToPos()
   //{
        //extendHookLift.setPosition(LIFT_TO_POS);
    //}
    public void retractHook()
    {
        extendHookLift.setPower(-SERVO_POWER);
    }
    public void liftRobot()
    {
        winch.setPower(1);
    }

    public void liftRobot(int POWER, int DIST)
    {
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winch.setTargetPosition(DIST);
        while(winch.getCurrentPosition() >= winch.getTargetPosition() - 7 &&
                winch.getCurrentPosition() <=  winch.getTargetPosition() + 7)
        {
            winch.setPower(POWER);
            extendHookLift.setPower(SERVO_POWER);
        }
        winch.setPower(0);
    }

    public void lowerRobot()
    {
        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //later change this to go to position 0 on encoder counts
        winch.setPower(-1);
    }

    public void outTelemetry()
    {
        base.outTelemetry.write("----------------HOOK LIFT---------------");
        base.outTelemetry.addData("Winch Power", winch.getPower());
        base.outTelemetry.addData("Winch Position", winch.getCurrentPosition());
        base.outTelemetry.addData("ExtendHook Power: ", extendHookLift.getPower());
        base.outTelemetry.write("-------------END HOOK LIFT ---------");
        base.outTelemetry.newLine();
    }

    @Override
    public void stop()
    {
        winch.setPower(0);
    }
}
