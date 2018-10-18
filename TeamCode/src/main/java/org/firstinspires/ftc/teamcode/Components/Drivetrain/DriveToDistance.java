package org.firstinspires.ftc.teamcode.Components.Drivetrain;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotCommand;

//This class will create the DriveTo command, which will be utilized all throughout the autonomous portion.
public class DriveToDistance extends RobotCommand
{
    private Drivetrain _drivetrain;

    private double distance = 0;
    private double speed = 0;

    private int BUFFER = 7;

    private boolean _busy = false;
    private boolean endCommand = false;

    private long TIMEOUT = 10000;

    //default constructor
    public DriveToDistance()
    {

    }
    public DriveToDistance(Drivetrain DRIVETRAIN)
    {
        _drivetrain = DRIVETRAIN;
    }
    public void init (Drivetrain DRIVETRAIN)
    {
        _drivetrain = DRIVETRAIN;
    }
    public void goTo(double DIST, double SPEED)
    {
        distance = DIST;
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
        if(_drivetrain.getEncoderMode() != DcMotor.RunMode.RUN_TO_POSITION)
        {
            _drivetrain.encoderToPos();
        }

        _busy = true;

        _drivetrain.setAllMotorPower(speed);

        long startTime = SystemClock.currentThreadTimeMillis();

        while( !endCommand && _drivetrain.isBusy() && _drivetrain.base().opMode.opModeIsActive() && System.currentTimeMillis() - startTime < TIMEOUT)
        {
            //Keep running! :D
        }

        _drivetrain.stop();

        _drivetrain.encoderOn();

        //Command is finished, for teleop now manually drive the robot, for autonomous supply more commands.
        _busy = false;

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