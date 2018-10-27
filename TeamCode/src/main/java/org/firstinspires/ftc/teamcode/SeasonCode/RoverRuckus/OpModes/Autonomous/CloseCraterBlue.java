package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.RoverVuforia;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;


@Autonomous(name= "Blue: CRATER")
public class CloseCraterBlue extends LinearOpMode
{

    public Base _base = new Base();
    //private RoverVuforia _vuforia = new RoverVuforia();

    //Hold state of where gold block is sitting
    private enum BlocKState
    {
        LEFT, MIDDLE, RIGHT, UNCERTAIN
    }

    @Override
    public void runOpMode()
    {
        BlocKState _block = BlocKState.UNCERTAIN;
        _base.init(hardwareMap, this);
        _base.imu.calibrateTo(95);//TWEAK THIS IDK

        waitForStart();

        //telemetry.addData("Test", ten);

        _base.drivetrain.driveTo.goTo(19, .4);
        _base.drivetrain.driveTo.runSequentially();
        telemetry.addData("Check distance set Back Left,",  _base.drivetrain.getTargetEncoderCounts()[0]);
        telemetry.addData("Check distance set Back Right,", _base.drivetrain.getTargetEncoderCounts()[1]);
        telemetry.addData("Check distance set Front Left,", _base.drivetrain.getTargetEncoderCounts()[2]);
        telemetry.addData("Check distance set Front Right,", _base.drivetrain.getTargetEncoderCounts()[3]);
        telemetry.addData("Check current set Back Left,",  _base.drivetrain.getEncoderCounts()[0]);
        telemetry.addData("Check current set Back Right,", _base.drivetrain.getEncoderCounts()[1]);
        telemetry.addData("Check current set Front Left,", _base.drivetrain.getEncoderCounts()[2]);
        telemetry.addData("Check current set Front Right,", _base.drivetrain.getEncoderCounts()[3]);
        telemetry.update();

//        _base.drivetrain.driveTo.goTo(25, .65);
//        _base.drivetrain.driveTo.runSequentially();

        _base.drivetrain.turnTo.goTo(129, .52);
        _base.drivetrain.turnTo.runSequentially();
        telemetry.addData("Angle Z: ", _base.imu.zAngle());
        telemetry.addData("Angle X: ", _base.imu.xAngle());
        telemetry.addData("Angle Y: ", _base.imu.yAngle());
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

        _base.drivetrain.driveTo.goTo(500, .3);
        _base.drivetrain.driveTo.runSequentially();
//
        sleep(2000); //test
//
////        while(true) //ooooffff don't use these if possible, in this case it is necessary though
////        {
////
////        }
//
        _base.drivetrain.stop();
    }



}
