package org.firstinspires.ftc.teamcode.Components.Drivetrain;

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

    private final static double DESIRED_OFFSET = 3.9;
    private final static double FRONT_DESIRED_OFFSET = 32.4;

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
        long startTime = System.currentTimeMillis();

        while( !endCommand && _drivetrain.isBusy() && _drivetrain.base().opMode.opModeIsActive()) //&& System.currentTimeMillis() - startTime < TIMEOUT)
        {
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
        long startTime = System.currentTimeMillis();
        while( !endCommand && _drivetrain.isBusy() && _drivetrain.base().opMode.opModeIsActive() && Math.abs(System.currentTimeMillis() - startTime) < TIMEOUT)
        {
//            _drivetrain.base().outTelemetry.addData("Start Time: ", startTime);
//            _drivetrain.base().outTelemetry.addData("Current System Time ", System.currentTimeMillis());
//            _drivetrain.base().outTelemetry.addData("Timeout ", TIMEOUT);
            _drivetrain.base().outTelemetry.update();

            //Keep running! :D
        }

        _drivetrain.setAllMotorPower(0);

        _drivetrain.encoderOn();

        //Command is finished, for teleop now manually drive the robot, for autonomous supply more commands.
        _busy = false;


    }


    public boolean runSequentiallyBlockDetection(double SECONDS_TIMEOUT){
        boolean found = false;
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
        long startTime = System.currentTimeMillis();
        while( !endCommand && _drivetrain.isBusy() && _drivetrain.base().opMode.opModeIsActive() && Math.abs(System.currentTimeMillis() - startTime) < TIMEOUT)
        {
            _drivetrain.base().outTelemetry.addData("Start Time: ", startTime);
            _drivetrain.base().outTelemetry.addData("Current System Time ", System.currentTimeMillis());
            _drivetrain.base().outTelemetry.addData("Timeout ", TIMEOUT);
            _drivetrain.base().outTelemetry.update();



            //Keep running! :D
        }

        _drivetrain.setAllMotorPower(0);

        _drivetrain.encoderOn();

        //Command is finished, for teleop now manually drive the robot, for autonomous supply more commands.
        _busy = false;
        return found;
    }

//    public void runKeepingDistance(){
//
//        double currentRange;
//        double error;
//        double correctionSpeed = 0.2;
//        double leftPowerMultiplier = 1;
//        double rightPowerMultiplier = 1;
//
//        currentRange =  _drivetrain.right.distance();
//
//        error = Math.abs(currentRange - DESIRED_OFFSET);
//
//        if(_drivetrain.getEncoderMode() != DcMotor.RunMode.RUN_TO_POSITION)
//        {
//            _drivetrain.encoderToPos();
//        }
//
//        if ( currentRange > DESIRED_OFFSET){
//            leftPowerMultiplier = 1 + (error * correctionSpeed);
//        }
//
//        else if (currentRange < DESIRED_OFFSET){
//            rightPowerMultiplier = 1 + (error * correctionSpeed);
//        }
//
//        _drivetrain.backLeft().setTargetPosition((int)(distance * COUNTS_PER_INCH) + _drivetrain.backLeft().getCurrentPosition());
//        _drivetrain.backRight().setTargetPosition((int)(distance * COUNTS_PER_INCH)+ _drivetrain.backRight().getCurrentPosition());
//        _drivetrain.frontLeft().setTargetPosition((int)(distance * COUNTS_PER_INCH) + _drivetrain.frontLeft().getCurrentPosition());
//        _drivetrain.frontRight().setTargetPosition((int)(distance * COUNTS_PER_INCH) + _drivetrain.frontRight().getCurrentPosition());
//        _busy = true;
//
//        _drivetrain.backLeft().setPower(speed * leftPowerMultiplier);
//        _drivetrain.frontLeft().setPower(speed * leftPowerMultiplier);
//
//        _drivetrain.backRight().setPower(speed * rightPowerMultiplier);
//        _drivetrain.frontRight().setPower(speed * rightPowerMultiplier);
//
//
//        while( !endCommand && _drivetrain.isBusy() && _drivetrain.base().opMode.opModeIsActive()) //&& System.currentTimeMillis() - startTime < TIMEOUT)
//        {
//            leftPowerMultiplier = 1;
//            rightPowerMultiplier = 1;
//
//            currentRange =  _drivetrain.right_range.distance();
//
//            if (currentRange < 10){
//
//                error = Math.abs(currentRange - DESIRED_OFFSET);
//
//                if ( currentRange > DESIRED_OFFSET){
//                    leftPowerMultiplier = 1 + (error * correctionSpeed);
//                }
//
//                else if (currentRange < DESIRED_OFFSET){
//                    rightPowerMultiplier = 1 + (error * correctionSpeed);
//                }
//
//                _drivetrain.backLeft().setPower(speed * leftPowerMultiplier);
//                _drivetrain.frontLeft().setPower(speed * leftPowerMultiplier);
//
//                _drivetrain.backRight().setPower(speed * rightPowerMultiplier);
//                _drivetrain.frontRight().setPower(speed * rightPowerMultiplier);
//                _drivetrain.base().outTelemetry.addData("Range Sensor: ", currentRange);
//                _drivetrain.base().outTelemetry.update();
//                try{
//                    Thread.sleep(10);}
//                catch(Exception e){e.printStackTrace();}
//            }
//
//
//        }
//
//        _drivetrain.setAllMotorPower(0);
//
//        _drivetrain.encoderOn();
//
//        //Command is finished, for teleop now manually drive the robot, for autonomous supply more commands.
//        _busy = false;
//
//    }

    public boolean runStopIfTouch(){

        boolean bumped = false;

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

        long startTime = System.currentTimeMillis();

        while( !endCommand && _drivetrain.isBusy() && _drivetrain.base().opMode.opModeIsActive()) //&& System.currentTimeMillis() - startTime < TIMEOUT)
        {
            if (_drivetrain.touch.isPressed()){
                bumped = true;
                break;
            }
            if(_drivetrain.front_range.distance(DistanceUnit.INCH) < 10){
                _drivetrain.setAllMotorPower(0.3);
            }
        }

        _drivetrain.setAllMotorPower(0);

        _drivetrain.encoderOn();

        //Command is finished, for teleop now manually drive the robot, for autonomous supply more commands.
        _busy = false;
        return bumped;

    }

    public boolean runStopIfTouch(double SECONDS_TIMEOUT)
    {

        boolean bumped = false;

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
        long startTime = System.currentTimeMillis();

        while( !endCommand && _drivetrain.isBusy() && _drivetrain.base().opMode.opModeIsActive() && Math.abs(System.currentTimeMillis() - startTime) < TIMEOUT)
        {
            if (_drivetrain.touch.isPressed()){

                bumped = true;
                break;
            }
            if (_drivetrain.front_range.distance(DistanceUnit.INCH) < 10){
                _drivetrain.setAllMotorPower(0.3);
            }
        }

        _drivetrain.setAllMotorPower(0);

        _drivetrain.encoderOn();

        //Command is finished, for teleop now manually drive the robot, for autonomous supply more commands.
        _busy = false;

        return bumped;
    }

    public boolean runStopIfDist()
    {
        boolean withinDist = false;
        double currentRange = 0;
        double distFrom = 0;
        double correctionSpeed = 0.2;
        double leftPowerMultiplier = 1;
        double rightPowerMultiplier = 1;

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
        long startTime = System.currentTimeMillis();

        while( !endCommand && _drivetrain.isBusy() && _drivetrain.base().opMode.opModeIsActive())
        {

            currentRange =  _drivetrain.front_range.distance(DistanceUnit.INCH);

            if ( currentRange > FRONT_DESIRED_OFFSET ) //+ 8 && currentRange < FRONT_DESIRED_OFFSET )
            {
                leftPowerMultiplier *= .75;
                rightPowerMultiplier *= .75;
                withinDist = false;
            }
            else
            {
                withinDist = true;
                break;
            }

            _drivetrain.backLeft().setPower(speed * leftPowerMultiplier);
            _drivetrain.frontLeft().setPower(speed * leftPowerMultiplier);

            _drivetrain.backRight().setPower(speed * rightPowerMultiplier);
            _drivetrain.frontRight().setPower(speed * rightPowerMultiplier);
            _drivetrain.base().outTelemetry.addData("Front Range Sensor: ", currentRange);
            _drivetrain.base().outTelemetry.update();
            try{
                Thread.sleep(10);}
            catch(Exception e){e.printStackTrace();}
        }

        _drivetrain.setAllMotorPower(0);

        _drivetrain.encoderOn();

        //Command is finished, for teleop now manually drive the robot, for autonomous supply more commands.
        _busy = false;

        return withinDist;
    }

    public boolean runStopIfDist(double SECONDS_TIMEOUT)
    {
        boolean withinDist = false;
        double currentRange = 0;
        double distFrom = 0;
        double correctionSpeed = 0.2;
        double powerMultiplier = 1;


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
        //Begin timing for timeout function
        long startTime = System.currentTimeMillis();

        while( !endCommand && _drivetrain.isBusy() && _drivetrain.base().opMode.opModeIsActive() && Math.abs(System.currentTimeMillis() - startTime) < TIMEOUT)
        {

            //Read in range from sensor
            currentRange =  _drivetrain.front_range.distance(DistanceUnit.INCH);

            //Begin Decelerating
            if ( currentRange > FRONT_DESIRED_OFFSET && currentRange < FRONT_DESIRED_OFFSET + 20)
            {
                powerMultiplier = Math.abs(currentRange - FRONT_DESIRED_OFFSET) * .05;
                withinDist = false;
            }
            else if(currentRange < FRONT_DESIRED_OFFSET && currentRange > 30)
            {
                withinDist = true;
                break;
            }
            double minimumPower = 0.07;
            if(powerMultiplier * speed <= minimumPower)
            {
                _drivetrain.backLeft().setPower(minimumPower);
                _drivetrain.frontLeft().setPower(minimumPower);

                _drivetrain.backRight().setPower(minimumPower);
                _drivetrain.frontRight().setPower(minimumPower);
            }
            //Accelerate or decelerate depending on range
            else {
                _drivetrain.backLeft().setPower(speed * powerMultiplier);
                _drivetrain.frontLeft().setPower(speed * powerMultiplier);

                _drivetrain.backRight().setPower(speed * powerMultiplier);
                _drivetrain.frontRight().setPower(speed * powerMultiplier);
            }
            _drivetrain.base().outTelemetry.addData("Front Range Sensor: ", currentRange);
            _drivetrain.base().outTelemetry.update();
            try{
                Thread.sleep(10);}
            catch(Exception e){e.printStackTrace();}
        }

        _drivetrain.setAllMotorPower(0);

        _drivetrain.encoderOn();

        //Command is finished, for teleop now manually drive the robot, for autonomous supply more commands.
        _busy = false;

        return withinDist;
    }

    public boolean runStopIfDist(double frontDistance, double SECONDS_TIMEOUT)
    {
        boolean withinDist = false;
        double currentRange = 0;
        double distFrom = 0;
        double correctionSpeed = 0.2;
        double powerMultiplier = 1;


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
        //Begin timing for timeout function
        long startTime = System.currentTimeMillis();

        while( !endCommand && _drivetrain.isBusy() && _drivetrain.base().opMode.opModeIsActive() && Math.abs(System.currentTimeMillis() - startTime) < TIMEOUT)
        {

            //Read in range from sensor
            currentRange =  _drivetrain.front_range.distance(DistanceUnit.INCH);

            //Begin Decelerating
            if ( currentRange > frontDistance && currentRange < frontDistance + 5)
            {
                powerMultiplier = Math.abs(currentRange - frontDistance) * .05;
                withinDist = false;
            }
            else if(currentRange < frontDistance && currentRange > frontDistance - 7)
            {
                withinDist = true;
                break;
            }
            double minimumPower = 0.07;
            if(powerMultiplier * speed <= minimumPower)
            {
                _drivetrain.backLeft().setPower(minimumPower);
                _drivetrain.frontLeft().setPower(minimumPower);

                _drivetrain.backRight().setPower(minimumPower);
                _drivetrain.frontRight().setPower(minimumPower);
            }
            //Accelerate or decelerate depending on range
            else {
                _drivetrain.backLeft().setPower(speed * powerMultiplier);
                _drivetrain.frontLeft().setPower(speed * powerMultiplier);

                _drivetrain.backRight().setPower(speed * powerMultiplier);
                _drivetrain.frontRight().setPower(speed * powerMultiplier);
            }
            _drivetrain.base().outTelemetry.addData("Front Range Sensor: ", currentRange);
            _drivetrain.base().outTelemetry.update();
            try{
                Thread.sleep(10);}
            catch(Exception e){e.printStackTrace();}
        }

        _drivetrain.setAllMotorPower(0);

        _drivetrain.encoderOn();

        //Command is finished, for teleop now manually drive the robot, for autonomous supply more commands.
        _busy = false;

        return withinDist;
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
