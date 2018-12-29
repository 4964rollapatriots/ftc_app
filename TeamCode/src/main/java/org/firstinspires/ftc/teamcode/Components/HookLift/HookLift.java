package org.firstinspires.ftc.teamcode.Components.HookLift;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.RobotComponent;

import static com.qualcomm.hardware.hitechnic.HiTechnicNxtIrSeekerSensor.DIRECTION;

/**
 * Created by pmkf2 on 10/17/2018.
 */

public class HookLift extends RobotComponent
{
    private DcMotor winch;
    private CRServo extendHookLift;

    private int LIFT_TO_POS = 50;

    public void init(RobotBase BASE)
    {
        super.init(BASE);
        winch = mapper.mapMotor("winch", DcMotorSimple.Direction.FORWARD);
        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendHookLift = mapper.mapCRServo("lift", CRServo.Direction.FORWARD);
    }


        public void lowerRobot()
        {
            winch.setPower(-1);
        }
        //Run the actual collector motor
        public void liftRobot()
        {
            winch.setPower(1);
        }

        public void extendHook()
        {
            extendHookLift.setPower(1);
        }
        public void extendHook(double power){
            extendHookLift.setPower(power);
        }

        public void stopCRServo()
        {
            extendHookLift.setPower(0);
        }
        public void retractHook()
        {
            extendHookLift.setPower(-1);
        }
        public void outTelemetry()
        {
            base.outTelemetry.write("----------COLLECTOR SYSTEM-----------");
            base.outTelemetry.addData("winch Power: ", winch.getPower());
            base.outTelemetry.addData("Hook Extension Power: ", extendHookLift.getPower());
            base.outTelemetry.write("----------END COLLECTOR SYSTEM---------");
            base.outTelemetry.newLine();
        }
        @Override
        public void stop()

        {
            winch.setPower(0);
            extendHookLift.setPower(0);
        }
    }

