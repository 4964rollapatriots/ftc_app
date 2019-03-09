package org.firstinspires.ftc.teamcode.Components.HookLift;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.RobotComponent;
import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.REVMagnetic;

import static com.qualcomm.hardware.hitechnic.HiTechnicNxtIrSeekerSensor.DIRECTION;

/**
 * Created by pmkf2 on 10/17/2018.
 */

public class HookLift extends RobotComponent
{
    private DcMotor winch;
    private CRServo extendHookLift;
    private CRServo hook;
    public REVMagnetic hookLimitSwitch;
    public REVMagnetic liftLimitSwitch;

    private int LIFT_TO_POS = 50;

    long startTime = 0;

    public void init(RobotBase BASE, REVMagnetic hookSwitch, REVMagnetic liftSwitch)
    {
        super.init(BASE);
        hookLimitSwitch = hookSwitch;
        liftLimitSwitch = liftSwitch;
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
            long startTime = System.currentTimeMillis();
            while(!liftLimitSwitch.isClose() &&  Math.abs(System.currentTimeMillis() - startTime) < TIME)
            {
                winch.setPower(-1);
            }
            winch.setPower(0);
        }

        public void lowerRobotWithEncoders(int encoders){
            winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            winch.setTargetPosition(encoders);
            while(Math.abs(winch.getCurrentPosition()-winch.getTargetPosition())>50){
                winch.setPower(-1);
            }
            winch.setPower(0);
            winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        //Run the actual collector motor
        public void liftRobot()
        {
            winch.setPower(1);
            extendHookLift.setPower(-.30);
        }

        public void startLiftingTime(long time){
            startTime = time;
        }

        public boolean extendHookTeleop(int  TIMEOUT){

            if(!liftLimitSwitch.isClose() &&  Math.abs(System.currentTimeMillis() - startTime) < TIMEOUT)
            {
                extendHookLift.setPower(1);
                base.outTelemetry.addData("Powering on now!!!!", true);
                base.outTelemetry.update();

                return false;
            }
            else{
                extendHook(0);
                return true;
            }

        }

        public void extendHookWithSensor()
        {
            long startTime = System.currentTimeMillis();
            while(!liftLimitSwitch.isClose() &&  Math.abs(System.currentTimeMillis() - startTime) < 4500)
            {
                extendHookLift.setPower(1);
            }
            stopHook();

        }
        public void extendHookManual()
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
        public void openHook(int TIMEOUT){
            long startTime = System.currentTimeMillis();
            while(!hookLimitSwitch.isClose() &&  Math.abs(System.currentTimeMillis() - startTime) < TIMEOUT)
            {
                hook.setPower(1);
            }
            stopHook();
        }

    public void openHookandExtend(int time){
        hook.setPower(1);
        extendHookWithSensor();
        try
        {
            Thread.sleep(time);
        }
        catch(Exception ex)
        {
            ex.printStackTrace();
        }
        stopHook();
        extendHook(0);
    }
        public void closeHook(){
            hook.setPower(-1);
        }
        public void closeHook(int time){
            hook.setPower(-1);
            try
            {
                Thread.sleep(2100);
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
            base.outTelemetry.addData("HOOK POWER", hook.getPower());
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

