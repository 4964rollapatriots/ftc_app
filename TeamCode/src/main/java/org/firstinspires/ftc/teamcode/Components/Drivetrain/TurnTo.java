package org.firstinspires.ftc.teamcode.Components.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotCommand;
import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.REVIMU;
import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.Util;

public class TurnTo extends RobotCommand
{

    private Drivetrain _drivetrain;

    private REVIMU _imu;

    private boolean endCommand = false;
    private boolean _busy = false;

    private double targetAngle = 0.0;
    private double maxSpeed = 0.0;
    private double minSpeed = .30;
    private double SPEED_MULT = 3.0;



    private long TIMEOUT = 10000;

    private int BUFFER = 5;


    //default constructor
    public TurnTo()
    {
        
    }
    public TurnTo(Drivetrain DRIVETRAIN, REVIMU IMU)
    {
        _drivetrain = DRIVETRAIN;
        _imu = IMU;
    }

    public void goTo(double ANGLE, double MAXSPEED)
    {
        targetAngle = ANGLE;
        maxSpeed = MAXSPEED;
    }

    public void goTo(double ANGLE, double MAXSPEED, double TIMEOUT)
    {
        targetAngle = ANGLE;
        maxSpeed = MAXSPEED;
        TIMEOUT = TIMEOUT;
    }

    @Override
    public void runSequentially()
    {
        if(_drivetrain.getEncoderMode() != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            _drivetrain.encoderOn();
        }

        _imu.setAngle();

        //Init heading stored from imu to variable.
        double initHeading = _imu.zAngle();

        //Tracks the current speed of the robot.
        double currSpeed = 0;

        //Formula for calculating error.
        double error = Util.angleError((int)initHeading, (int)targetAngle);

        //Find's initial error will be used in calculations
        double initError = error;

        //Finding the distance modifier
        double distModfication = Math.abs(error) / 180;

        //Start time of angle movements, will be used for timeouts
        long startTime = System.currentTimeMillis();

        _busy = true;

        _drivetrain.setCurrState(Drivetrain.State.FORWARD_FAST);

        while(Math.abs(error) > BUFFER && System.currentTimeMillis() - startTime < TIMEOUT
                && !endCommand && _drivetrain.base().opMode.opModeIsActive())
        {
            _imu.setAngle();

            error = Util.angleError((int)_imu.zAngle(), (int)targetAngle);

            currSpeed = (error/initError) * SPEED_MULT * distModfication;

            if(currSpeed < minSpeed)
            {
                currSpeed = minSpeed;
            }

            if(currSpeed > maxSpeed)
            {
                currSpeed = maxSpeed;
            }



        }

        _drivetrain.stop();

        _busy = false;
    }

    @Override
    public void runParallel()
    {
        //Thread t;
    }

    @Override
    public void stop()
    {
        endCommand = true;
    }
}
