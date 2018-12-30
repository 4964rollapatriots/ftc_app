package org.firstinspires.ftc.teamcode.Components.Drivetrain;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotCommand;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//This class will create the DriveTo command, which will be utilized all throughout the autonomous portion.
public class DriveToDistance extends RobotCommand
{
    private Drivetrain _drivetrain;

    private double distance = 0;
    private double speed = 0;

    private int BUFFER = 7;

    private final static double DESIRED_OFFSET = 2;

    private boolean _busy = false;
    private boolean endCommand = false;

    private long TIMEOUT = 10000;
    public double COUNTS_PER_INCH = 40.78651685 * .80; //complete guess, need to calculate
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
    public void goTo(double INCHES, double SPEED)
    {
        distance = INCHES;
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


        _drivetrain.backLeft().setTargetPosition((int)(distance * COUNTS_PER_INCH) + _drivetrain.backLeft().getCurrentPosition());
        _drivetrain.backRight().setTargetPosition((int)(distance * COUNTS_PER_INCH)+ _drivetrain.backRight().getCurrentPosition());
        _drivetrain.frontLeft().setTargetPosition((int)(distance * COUNTS_PER_INCH) + _drivetrain.frontLeft().getCurrentPosition());
        _drivetrain.frontRight().setTargetPosition((int)(distance * COUNTS_PER_INCH) + _drivetrain.frontRight().getCurrentPosition());
        _busy = true;

        _drivetrain.setAllMotorPower(speed);
        long startTime = SystemClock.currentThreadTimeMillis();

        while( !endCommand && _drivetrain.isBusy() && _drivetrain.base().opMode.opModeIsActive()) //&& System.currentTimeMillis() - startTime < TIMEOUT)
        {
            //Keep running! :D
        }

        _drivetrain.setAllMotorPower(0);

        _drivetrain.encoderOn();

        //Command is finished, for teleop now manually drive the robot, for autonomous supply more commands.
        _busy = false;


    }

    public void runSequentially(double SECONDS_TIMEOUT)
    {
        TIMEOUT = (long)(SECONDS_TIMEOUT * 1000);
        if(_drivetrain.getEncoderMode() != DcMotor.RunMode.RUN_TO_POSITION)
        {
            _drivetrain.encoderToPos();
        }


        _drivetrain.backLeft().setTargetPosition((int)(distance * COUNTS_PER_INCH) + _drivetrain.backLeft().getCurrentPosition());
        _drivetrain.backRight().setTargetPosition((int)(distance * COUNTS_PER_INCH)+ _drivetrain.backRight().getCurrentPosition());
        _drivetrain.frontLeft().setTargetPosition((int)(distance * COUNTS_PER_INCH) + _drivetrain.frontLeft().getCurrentPosition());
        _drivetrain.frontRight().setTargetPosition((int)(distance * COUNTS_PER_INCH) + _drivetrain.frontRight().getCurrentPosition());
        _busy = true;

        _drivetrain.setAllMotorPower(speed);
        long startTime = SystemClock.currentThreadTimeMillis();

        while( !endCommand && _drivetrain.isBusy() && _drivetrain.base().opMode.opModeIsActive() && System.currentTimeMillis() - startTime < TIMEOUT)
        {
            //Keep running! :D
        }

        _drivetrain.setAllMotorPower(0);

        _drivetrain.encoderOn();

        //Command is finished, for teleop now manually drive the robot, for autonomous supply more commands.
        _busy = false;


    }

    public void runKeepingDistance(){

        double currentRange;
        double error;
        double correctionSpeed = 0.2;
        double leftPowerMultiplier = 1;
        double rightPowerMultiplier = 1;

        currentRange =  _drivetrain.range.distance(DistanceUnit.INCH);

        error = Math.abs(currentRange - DESIRED_OFFSET);

        if(_drivetrain.getEncoderMode() != DcMotor.RunMode.RUN_TO_POSITION)
        {
            _drivetrain.encoderToPos();
        }

        if ( currentRange > DESIRED_OFFSET){
            leftPowerMultiplier = 1 + (error * correctionSpeed);
        }

        else if (currentRange < DESIRED_OFFSET){
            rightPowerMultiplier = 1 + (error * correctionSpeed);
        }

        _drivetrain.backLeft().setTargetPosition((int)(distance * COUNTS_PER_INCH) + _drivetrain.backLeft().getCurrentPosition());
        _drivetrain.backRight().setTargetPosition((int)(distance * COUNTS_PER_INCH)+ _drivetrain.backRight().getCurrentPosition());
        _drivetrain.frontLeft().setTargetPosition((int)(distance * COUNTS_PER_INCH) + _drivetrain.frontLeft().getCurrentPosition());
        _drivetrain.frontRight().setTargetPosition((int)(distance * COUNTS_PER_INCH) + _drivetrain.frontRight().getCurrentPosition());
        _busy = true;

        _drivetrain.backLeft().setPower(speed * leftPowerMultiplier);
        _drivetrain.frontLeft().setPower(speed * leftPowerMultiplier);

        _drivetrain.backRight().setPower(speed * rightPowerMultiplier);
        _drivetrain.frontRight().setPower(speed * rightPowerMultiplier);


        while( !endCommand && _drivetrain.isBusy() && _drivetrain.base().opMode.opModeIsActive()) //&& System.currentTimeMillis() - startTime < TIMEOUT)
        {
            leftPowerMultiplier = 1;
            rightPowerMultiplier = 1;

            currentRange =  _drivetrain.range.distance(DistanceUnit.INCH);

            error = Math.abs(currentRange - DESIRED_OFFSET);

            if ( currentRange > DESIRED_OFFSET){
                leftPowerMultiplier = 1 + (error * correctionSpeed);
            }

            else if (currentRange < DESIRED_OFFSET){
                rightPowerMultiplier = 1 + (error * correctionSpeed);
            }

            _drivetrain.backLeft().setPower(speed * leftPowerMultiplier);
            _drivetrain.frontLeft().setPower(speed * leftPowerMultiplier);

            _drivetrain.backRight().setPower(speed * rightPowerMultiplier);
            _drivetrain.frontRight().setPower(speed * rightPowerMultiplier);

            try{
            Thread.sleep(10);}
            catch(Exception e){e.printStackTrace();}
        }

        _drivetrain.setAllMotorPower(0);

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
