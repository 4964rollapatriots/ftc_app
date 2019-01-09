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
    private CRServo hook;

    private int LIFT_TO_POS = 50;

    public void init(RobotBase BASE)
    {
        super.init(BASE);
        winch = mapper.mapMotor("winch", DcMotorSimple.Direction.REVERSE);
        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendHookLift = mapper.mapCRServo("lift", CRServo.Direction.REVERSE);
        hook = mapper.mapCRServo("hook", CRServo.Direction.FORWARD);
    }

        public void lowerRobot()
        {
            winch.setPower(-1);
        }

        public void lowerRobot(int TIME)
        {
            winch.setPower(-1);
            extendHookLift.setPower(0);
            try
            {
                Thread.sleep(TIME);
            }
            catch(Exception ex)
            {
                ex.printStackTrace();
            }
            winch.setPower(0);
        }

        public void lowerRobotWithEncoders(double)
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

        public void openHook(){
            hook.setPower(1);
        }
        public void openHook(int time){
            hook.setPower(1);
            try
            {
                Thread.sleep(time);
            }
            catch(Exception ex)
            {
                ex.printStackTrace();
            }
            stopHook();
        }
        public void closeHook(){
            hook.setPower(-1);
        }
        public void closeHook(int time){
            hook.setPower(-1);
            try
            {
                Thread.sleep(time);
            }
            catch(Exception ex)
            {
                ex.printStackTrace();
            }
            stopHook();
        }
        public void stopHook(){
            hook.setPower(0);
        }

        public int getWinchEncoders()
        {
            return winch.getCurrentPosition();
        }
        public void outTelemetry()
        {
            base.outTelemetry.write("----------LATCH SYSTEM-----------");
            base.outTelemetry.addData("winch Power: ", winch.getPower());
            base.outTelemetry.addData("Winch Encoders ", getWinchEncoders());
            base.outTelemetry.addData("Hook Extension Power: ", extendHookLift.getPower());
            base.outTelemetry.write("----------END LATCH SYSTEM---------");
            base.outTelemetry.newLine();
        }
        @Override
        public void stop()

        {
            winch.setPower(0);
            extendHookLift.setPower(0);
            hook.setPower(0);
        }
    }

