package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.UtilCV;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;

@Autonomous(name = "CORNER")

public class Corner extends LinearOpMode {
    //
    public Base _base = new Base();
    private UtilCV eye;

    private static int FIRST_TURN = 32;
    private static int SECOND_TURN = 50;
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
        eye = new UtilCV(hardwareMap);
        waitForStart();

        //telemetry.addData("Test", ten);
        sleep(500);
        if (eye.isAligned()){
            _block = blockState.MIDDLE;
            telemetry.addData("block is", _block);
        }
        if (_block == blockState.UNCERTAIN){
            driveToDistance(360 - FIRST_TURN, 0.18);
            sleep(500);
            if (eye.isAligned()){
                _block = blockState.RIGHT;
                telemetry.addData("block is", _block);
            }
        }
        if (_block == blockState.UNCERTAIN){
            turnToAngle(FIRST_TURN,0.2);
            sleep(500);
            if (eye.isAligned()){
                _block = blockState.LEFT;
                telemetry.addData("block is", _block);
            }
        }
        telemetry.addData("block state is ", _block);
        sleep(500);

        driveToDistance(-30,-0.5);

        driveToDistance(30,0.5);

        turnToAngle(SECOND_TURN, 0.5);

        driveToDistance(-37, -0.7);

        turnToAngle(360 - 45, 0.3);

        driveToDistance(-48, -0.6);

        //dump relic

        turnToAngle(135, 0.4);

        driveToDistance( -68, -0.9);
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
