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
    private double minSpeed = .14;
    private double SPEED_MULT = 2;
    private double SPEED_MULT_BLOCK = 2.3;
    private double ARC_SPEED_MULT = 2.0;



    private long TIMEOUT = 10000;

    private int BUFFER = 4;


    //default constructor
    public TurnTo()
    {
        
    }
    public TurnTo(Drivetrain DRIVETRAIN, REVIMU IMU)
    {
        _drivetrain = DRIVETRAIN;
        _imu = IMU;
    }

    public void     goTo(double ANGLE, double MAXSPEED)
    {
        if(ANGLE == 360)
            ANGLE = 0;
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

        double heading = 0;

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

        while(Math.abs(error) > BUFFER && Math.abs(System.currentTimeMillis() - startTime) < TIMEOUT
                && !endCommand && _drivetrain.base().opMode.opModeIsActive())
        {
            _imu.setAngle();
            heading = _imu.zAngle();
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

            if(currSpeed < minSpeed)
            {
                currSpeed = minSpeed;
            }

            _drivetrain.base().outTelemetry.addData("INIT ERROR: ", initError);
            _drivetrain.base().outTelemetry.addData("Init Heading :", initHeading);
            _drivetrain.base().outTelemetry.addData("TARGET ANGLE: ", targetAngle);
            _drivetrain.base().outTelemetry.addData("Current ANGLE: ", heading);
            _drivetrain.base().outTelemetry.addData("distModification: ", distModfication);
            _drivetrain.base().outTelemetry.addData("ERROR: ", error);
            _drivetrain.base().outTelemetry.addData("CURRSPEED: ", currSpeed);
            //_drivetrain.outTelemetry();
            _drivetrain.base().outTelemetry.update();

            _drivetrain.run(0.0, Math.abs(error) / -error * currSpeed, false, 1);

        }

        _drivetrain.stop();

        _busy = false;
    }


    public void runSequentially(double SECONDS_TIMEOUT)
    {
        TIMEOUT = (long)(SECONDS_TIMEOUT * 1000);
        if(_drivetrain.getEncoderMode() != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            _drivetrain.encoderOn();
        }

        _imu.setAngle();

        //Init heading stored from imu to variable.
        double initHeading = _imu.zAngle();

        double heading = 0;

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
            heading = _imu.zAngle();
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

            if(currSpeed < minSpeed)
            {
                currSpeed = minSpeed;
            }

            _drivetrain.base().outTelemetry.addData("INIT ERROR: ", initError);
            _drivetrain.base().outTelemetry.addData("Init Heading :", initHeading);
            _drivetrain.base().outTelemetry.addData("TARGET ANGLE: ", targetAngle);
            _drivetrain.base().outTelemetry.addData("Current ANGLE: ", heading);
            _drivetrain.base().outTelemetry.addData("distModification: ", distModfication);
            _drivetrain.base().outTelemetry.addData("ERROR: ", error);
            _drivetrain.base().outTelemetry.addData("CURRSPEED: ", currSpeed);
            //_drivetrain.outTelemetry();
            _drivetrain.base().outTelemetry.update();

            _drivetrain.run(0.0, Math.abs(error) / -error * currSpeed, false,1);

        }

        _drivetrain.stop();

        _busy = false;
    }


    public void runSequentially(double BUFFER_PARAM, double SECONDS_TIMEOUT)
    {
        TIMEOUT = (long)(SECONDS_TIMEOUT * 1000);
        if(_drivetrain.getEncoderMode() != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            _drivetrain.encoderOn();
        }

        _imu.setAngle();

        //Init heading stored from imu to variable.
        double initHeading = _imu.zAngle();

        double heading = 0;

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

        while(Math.abs(error) > BUFFER_PARAM && System.currentTimeMillis() - startTime < TIMEOUT
                && !endCommand && _drivetrain.base().opMode.opModeIsActive())
        {
            _imu.setAngle();
            heading = _imu.zAngle();
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

            if(currSpeed < minSpeed)
            {
                currSpeed = minSpeed;
            }

            _drivetrain.base().outTelemetry.addData("INIT ERROR: ", initError);
            _drivetrain.base().outTelemetry.addData("Init Heading :", initHeading);
            _drivetrain.base().outTelemetry.addData("TARGET ANGLE: ", targetAngle);
            _drivetrain.base().outTelemetry.addData("Current ANGLE: ", heading);
            _drivetrain.base().outTelemetry.addData("distModification: ", distModfication);
            _drivetrain.base().outTelemetry.addData("ERROR: ", error);
            _drivetrain.base().outTelemetry.addData("CURRSPEED: ", currSpeed);
            //_drivetrain.outTelemetry();
            _drivetrain.base().outTelemetry.update();

            _drivetrain.run(0.0, Math.abs(error) / -error * currSpeed, false,1);

        }

        _drivetrain.stop();

        _busy = false;
    }
    public boolean teleopRunSequentially(int BUFFER_PARAM, double SECONDS_TIMEOUT)
    {
        TIMEOUT = (long)(SECONDS_TIMEOUT * 1000);
        if(_drivetrain.getEncoderMode() != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            _drivetrain.encoderOn();
        }

        _imu.setAngle();

        //Init heading stored from imu to variable.
        double initHeading = _imu.zAngle();

        double heading = 0;

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

        _imu.setAngle();
        heading = _imu.zAngle();
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
        if(currSpeed < minSpeed)
        {
            currSpeed = minSpeed;
        }


        _drivetrain.run(0.0, (Math.abs(error) / -error * currSpeed), false,1);

        if(Math.abs(System.currentTimeMillis() - startTime) < TIMEOUT
                && !endCommand && _drivetrain.base().opMode.opModeIsActive() && Math.abs(_imu.zAngle() - targetAngle) > 10)
        {
            return false;
        }
        else{
            _busy = false;
            return true;
        }


    }

    public void arcSequentially(double RATIO, double SECONDS_TIMEOUT)
    {
        TIMEOUT = (long)(SECONDS_TIMEOUT * 1000);
        if (_drivetrain.getEncoderMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            _drivetrain.encoderOn();
        }

        _imu.setAngle();

        //Init heading stored from imu to variable.
        double initHeading = _imu.zAngle();

        double heading = 0;

        //Tracks the current speed of the robot.
        double currSpeed = 0;

        //Formula for calculating error.
        double error = Util.angleError((int) initHeading, (int) targetAngle);

        //Find's initial error will be used in calculations
        double initError = error;

        //Finding the distance modifier
        double distModfication = Math.abs(error) / 180;

        //Start time of angle movements, will be used for timeouts
        long startTime = System.currentTimeMillis();

        _busy = true;

        _drivetrain.setCurrState(Drivetrain.State.FORWARD_FAST);

        while (Math.abs(error) > BUFFER && Math.abs(System.currentTimeMillis() - startTime) < TIMEOUT
                && !endCommand && _drivetrain.base().opMode.opModeIsActive()) {
            _imu.setAngle();
            heading = _imu.zAngle();
            error = Util.angleError((int) _imu.zAngle(), (int) targetAngle);
            currSpeed = (error / initError) * ARC_SPEED_MULT * distModfication;

            if (currSpeed < minSpeed) {
                currSpeed = minSpeed;
            }

            if (currSpeed > maxSpeed) {
                currSpeed = maxSpeed;
            }

            if (currSpeed < minSpeed) {
                currSpeed = minSpeed;
            }

            _drivetrain.base().outTelemetry.addData("INIT ERROR: ", initError);
            _drivetrain.base().outTelemetry.addData("Init Heading :", initHeading);
            _drivetrain.base().outTelemetry.addData("TARGET ANGLE: ", targetAngle);
            _drivetrain.base().outTelemetry.addData("Current ANGLE: ", heading);
            _drivetrain.base().outTelemetry.addData("distModification: ", distModfication);
            _drivetrain.base().outTelemetry.addData("ERROR: ", error);
            _drivetrain.base().outTelemetry.addData("CURRSPEED: ", currSpeed);
            //_drivetrain.outTelemetry();
            _drivetrain.base().outTelemetry.update();

            _drivetrain.arcRun(currSpeed+.03 ,currSpeed*RATIO);
        }
    }

    public void rightArcSequentially()
    {
        if (_drivetrain.getEncoderMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            _drivetrain.encoderOn();
        }

        _imu.setAngle();

        //Init heading stored from imu to variable.
        double initHeading = _imu.zAngle();

        double heading = 0;

        //Tracks the current speed of the robot.
        double currSpeed = 0;

        //Formula for calculating error.
        double error = Util.angleError((int) initHeading, (int) targetAngle);

        //Find's initial error will be used in calculations
        double initError = error;

        //Finding the distance modifier
        double distModfication = Math.abs(error) / 180;

        //Start time of angle movements, will be used for timeouts
        long startTime = System.currentTimeMillis();

        _busy = true;

        _drivetrain.setCurrState(Drivetrain.State.FORWARD_FAST);

        while (Math.abs(error) > BUFFER && System.currentTimeMillis() - startTime < TIMEOUT
                && !endCommand && _drivetrain.base().opMode.opModeIsActive()) {
            _imu.setAngle();
            heading = _imu.zAngle();
            error = Util.angleError((int) _imu.zAngle(), (int) targetAngle);
            currSpeed = (error / initError) * ARC_SPEED_MULT * distModfication;

            if (currSpeed < minSpeed) {
                currSpeed = minSpeed;
            }

            if (currSpeed > maxSpeed) {
                currSpeed = maxSpeed;
            }

            _drivetrain.arcRun(0,currSpeed*2);
        }
    }

    public void blockRunSequentially()
    {
        if(_drivetrain.getEncoderMode() != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            _drivetrain.encoderOn();
        }

        _imu.setAngle();

        //Init heading stored from imu to variable.
        double initHeading = _imu.zAngle();

        double heading = 0;

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
            heading = _imu.zAngle();
            error = Util.angleError((int)_imu.zAngle(), (int)targetAngle);
            currSpeed = (error/initError) * SPEED_MULT_BLOCK * distModfication;

            if(currSpeed < minSpeed)
            {
                currSpeed = minSpeed;
            }

            if(currSpeed > maxSpeed)
            {
                currSpeed = maxSpeed;
            }

            if(currSpeed < minSpeed)
            {
                currSpeed = minSpeed;
            }

//            _drivetrain.base().outTelemetry.addData("INIT ERROR: ", initError);
//            _drivetrain.base().outTelemetry.addData("Init Heading :", initHeading);
//            _drivetrain.base().outTelemetry.addData("TARGET ANGLE: ", targetAngle);
//            _drivetrain.base().outTelemetry.addData("Current ANGLE: ", heading);
//            _drivetrain.base().outTelemetry.addData("distModification: ", distModfication);
//            _drivetrain.base().outTelemetry.addData("ERROR: ", error);
//            _drivetrain.base().outTelemetry.addData("CURRSPEED: ", currSpeed);
            //_drivetrain.outTelemetry();
            _drivetrain.base().outTelemetry.update();

            _drivetrain.run(0.0, Math.abs(error) / -error * currSpeed, false,1);

        }

        _drivetrain.stop();

        _busy = false;
    }


    public void blockRunSequentially(int BUFFER_PARAM, double SECONDS_TIMEOUT)
    {
        TIMEOUT = (long)(SECONDS_TIMEOUT * 1000);
        if(_drivetrain.getEncoderMode() != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            _drivetrain.encoderOn();
        }

        _imu.setAngle();

        //Init heading stored from imu to variable.
        double initHeading = _imu.zAngle();

        double heading = 0;

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

        while(Math.abs(error) > BUFFER_PARAM && Math.abs(System.currentTimeMillis() - startTime) < TIMEOUT
                && !endCommand && _drivetrain.base().opMode.opModeIsActive())
        {
            _imu.setAngle();
            heading = _imu.zAngle();
            error = Util.angleError((int)_imu.zAngle(), (int)targetAngle);
            currSpeed = (error/initError) * SPEED_MULT_BLOCK * distModfication;

            if(currSpeed < minSpeed)
            {
                currSpeed = minSpeed;
            }

            if(currSpeed > maxSpeed)
            {
                currSpeed = maxSpeed;
            }

            if(currSpeed < minSpeed)
            {
                currSpeed = minSpeed;
            }

            _drivetrain.base().outTelemetry.addData("INIT ERROR: ", initError);
            _drivetrain.base().outTelemetry.addData("Init Heading :", initHeading);
            _drivetrain.base().outTelemetry.addData("TARGET ANGLE: ", targetAngle);
            _drivetrain.base().outTelemetry.addData("Current ANGLE: ", heading);
            _drivetrain.base().outTelemetry.addData("distModification: ", distModfication);
            _drivetrain.base().outTelemetry.addData("ERROR: ", error);
            _drivetrain.base().outTelemetry.addData("CURRSPEED: ", currSpeed);
            //_drivetrain.outTelemetry();
            _drivetrain.base().outTelemetry.update();

            _drivetrain.run(0.0, Math.abs(error) / -error * currSpeed, false,1);

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
