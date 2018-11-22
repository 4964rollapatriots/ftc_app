package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.UtilGoldDetector;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;

@Autonomous(name = "CORNER")

public class Corner extends LinearOpMode {
    //
    public Base _base = new Base();
    public UtilGoldDetector eye = new UtilGoldDetector(hardwareMap);

    private final static double FIRST_TURN_ANGLE = 35;
    private final static double MIDDLE_ANGLE = 10;

    //Hold state of where gold block is sitting
    private enum blockState
    {
        LEFT, MIDDLE, RIGHT, UNCERTAIN
    }

    @Override
    public void runOpMode()
    {
        blockState _block = blockState.UNCERTAIN;
        _base.init(hardwareMap, this);
        _base.imu.calibrateTo(0);//TWEAK THIS IDK

        waitForStart();

        //telemetry.addData("Test", ten);

        // sees if the block is initially aligned in the middle, if so it does not need to turn
        if (eye.isAligned()){
            _block = blockState.MIDDLE;
        }
        // if it does not see it, it will turn by single degrees until it reaches a set angle
        // if the block is on the left, the robot will see it as it turns
        else if (_block == blockState.UNCERTAIN){
            for (int i = 0; i < FIRST_TURN_ANGLE; i ++){
                _base.drivetrain.turnTo.goTo(i,0.5);
                _base.drivetrain.turnTo.runSequentially();
                if (eye.isAligned()){
                    _block = blockState.LEFT;
                    break;
                }
            }
        }
        //if the block is not on the left, then the robot turns to the initial zero degrees
        // it then turns slightly to the right to make sure that the ball is not in the middle
        // this is necessary because of the offset of our camera
        else if (_block == blockState.UNCERTAIN){
            _base.drivetrain.turnTo.goTo(0,0.5);
            _base.drivetrain.turnTo.runSequentially();
            for (int i = 0; i < MIDDLE_ANGLE; i ++){
                _base.drivetrain.turnTo.goTo(360-i, 0.5);
                _base.drivetrain.turnTo.runSequentially();
                if (eye.isAligned()){
                    _block = blockState.MIDDLE;
                    break;
                }
            }
        }
        // if and only if the robot has not yet found the ball,
        // it turns to the right by small increments until it reaches its target or sees the block
        else if (_block == blockState.UNCERTAIN){
            _base.drivetrain.turnTo.goTo(360 - MIDDLE_ANGLE,0.5);
            _base.drivetrain.turnTo.runSequentially();
            for (int i = 0; i < FIRST_TURN_ANGLE - MIDDLE_ANGLE; i ++){
                _base.drivetrain.turnTo.goTo(360-MIDDLE_ANGLE-i,0.5);
                _base.drivetrain.turnTo.runSequentially();
                if (eye.isAligned()){
                    _block = blockState.RIGHT;
                    break;
                }
            }
        }
        telemetry.addData("block state is", _block);
        telemetry.update();

        driveToDistance(20,0.15);

//        driveToDistance(30,0.5);
//
//        turnToAngle(SECOND_TURN, 0.5);
//
//        driveToDistance(-37, -0.7);
//
//        turnToAngle(360 - 45, 0.3);
//
//        driveToDistance(-48, -0.6);

        //dump relic

//        turnToAngle(135, 0.4);

        //driveToDistance( -30, -0.9);
//        telemetry.addData("Check distance set Back Left,",  _base.drivetrain.getTargetEncoderCounts()[0]);
//        telemetry.addData("Check distance set Back Right,", _base.drivetrain.getTargetEncoderCounts()[1]);
//        telemetry.addData("Check distance set Front Left,", _base.drivetrain.getTargetEncoderCounts()[2]);
//        telemetry.addData("Check distance set Front Right,", _base.drivetrain.getTargetEncoderCounts()[3]);
//        telemetry.addData("Check current set Back Left,",  _base.drivetrain.getEncoderCounts()[0]);
//        telemetry.addData("Check current set Back Right,", _base.drivetrain.getEncoderCounts()[1]);
//        telemetry.addData("Check current set Front Left,", _base.drivetrain.getEncoderCounts()[2]);
//        telemetry.addData("Check current set Front Right,", _base.drivetrain.getEncoderCounts()[3]);
//        telemetry.update();

//        _base.drivetrain.driveTo.goTo(25, .65);
//        _base.drivetrain.driveTo.runSequentially();

        // _base.drivetrain.turnTo.goTo(45, .52);
        //_base.drivetrain.turnTo.runSequentially();
        telemetry.addData("Angle Z: ", _base.imu.zAngle());
        telemetry.addData("Angle X: ", _base.imu.xAngle());
        telemetry.addData("Angle Y: ", _base.imu.yAngle());
        telemetry.addData("SPEED: ", _base.drivetrain.frontLeft().getPower());
        telemetry.update();
//        switch (_block)
//        {
//            case LEFT:
//                break;
//            case RIGHT:
//                break;
//            case MIDDLE:
//                break;
//            case UNCERTAIN:
//                break;
//        }
//        _base.drivetrain.turnTo.goTo(180, .70);
//        _base.drivetrain.turnTo.runSequentially();

        //_base.drivetrain.driveTo.goTo(500, .3);
        //_base.drivetrain.driveTo.runSequentially();
//
        _base.drivetrain.stop();
    }

    private void turnToAngle(double angle, double speed){
        _base.drivetrain.turnTo.goTo(angle, speed);
        _base.drivetrain.turnTo.runSequentially();

    }

    private void driveToDistance(double inches, double speed){
        _base.drivetrain.driveTo.goTo(inches, speed);
        _base.drivetrain.driveTo.runSequentially();
    }


}

