package org.firstinspires.ftc.teamcode.Components.HookLift;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotCommand;

//This class will create the DriveTo command, which will be utilized all throughout the autonomous portion.
public class DriveForTime extends RobotCommand
{
    private HookLift _lift;

    private double time = 0;
    private double speed = 0;


    private boolean _busy = false;
    private boolean endCommand = false;

    private long TIMEOUT = 10000;

    //default constructor
    public DriveForTime()
    {

    }
    public DriveForTime(HookLift HOOKLIFT)
    {
        _lift = HOOKLIFT;
    }
    public void init (HookLift HOOKLIFT)
    {

            _lift = HOOKLIFT;
    }
    public void runForTime(double TIME, double SPEED)
    {
        time = TIME;
        speed = SPEED;
    }

    @Override
    public void runParallel()
    {
        //404 :D
    }

    @Override
    public void runSequentially()
    {
//        if(_lift.getEncoderMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
//        {
//            _lift.encoderToPos();
//        }
//
//
//        _lift.setTargetPosition((int)(distance * COUNTS_PER_INCH) + _lift.backLeft().getCurrentPosition());
//        _busy = true;
//
//        _lift.setAllMotorPower(speed);
//        long startTime = SystemClock.currentThreadTimeMillis();
//
//        while( !endCommand && _lift.isBusy() && _lift.base().opMode.opModeIsActive()) //&& System.currentTimeMillis() - startTime < TIMEOUT)
//        {
//            //Keep running! :D
//        }
//
//        _lift.setAllMotorPower(0);
//
//        _lift.encoderOn();
//
//        //Command is finished, for teleop now manually drive the robot, for autonomous supply more commands.
//        _busy = false;
//

    }

    public Boolean isBusy()
    {
        return _busy;
    }

    @Override
    public void stop()
    {
        endCommand = true;
    }
}
