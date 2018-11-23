package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.UtilCV;
import org.firstinspires.ftc.teamcode.Components.MarkerDelivery.MarkerDelivery;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;

@Autonomous(name = "STANDARD CORNER")

public class StandardCrater extends LinearOpMode {
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

        sleep(500);
        if (eye.isAligned()){
            _block = blockState.MIDDLE;

            telemetry.addData("block is", _block);
            _base.outTelemetry.addData("Angle Z: ", _base.imu.zAngle());
            telemetry.update();
        }
        if (_block == blockState.UNCERTAIN){
            turnToAngle(360 - FIRST_TURN, 0.30);
            _base.outTelemetry.addData("Angle Z: ", _base.imu.zAngle());
            _base.outTelemetry.update();
            sleep(500);
            if (eye.isAligned()){
                _block = blockState.RIGHT;
                telemetry.addData("block is", _block);
                telemetry.update();
            }
        }
        if (_block == blockState.UNCERTAIN){
            turnToAngle(FIRST_TURN,0.30);
            _base.outTelemetry.addData("Angle Z: ", _base.imu.zAngle());
            _base.outTelemetry.update();
            sleep(500);
            if (eye.isAligned()){
                _block = blockState.LEFT;
                telemetry.addData("block is", _block);
                telemetry.update();
            }
        }
        telemetry.addData("block state is ", _block);
        sleep(500);

        driveToDistance(15,0.15);

        //driveToDistance(-15, .15);


//        turnToAngle(SECOND_TURN, 0.5);
        // _base.outTelemetry.addData("Angle Z: ", _base.imu.zAngle());
        //_base.outTelemetry.update();
//
//        driveToDistance(37, 0.7);
//
//        turnToAngle(360 - 45, 0.3);
        //_base.outTelemetry.addData("Angle Z: ", _base.imu.zAngle());
        //_base.outTelemetry.update();
//
//        driveToDistance(48, 0.6);


        //dump relic

//        turnToAngle(SOME ANGLE, .50);
        //_base.outTelemetry.addData("Angle Z: ", _base.imu.zAngle());
        //_base.outTelemetry.update();
//        _base.deliver.deliverMarker();
//
//        //re align with other crater
//        turnToAngle(360-45, .50);
        //_base.outTelemetry.addData("Angle Z: ", _base.imu.zAngle());
        //_base.outTelemetry.update();
//
//        driveToDistance(-100, .50);
        //DONE

        _base.outTelemetry.addData("Angle Z: ", _base.imu.zAngle());
        _base.outTelemetry.addData("SPEED: ", _base.drivetrain.frontLeft().getPower());
        _base.outTelemetry.update();

        _base.drivetrain.stop();
    }

    private void turnToAngle(double angle, double speed){
        _base.drivetrain.turnTo.goTo(angle, speed);
        _base.drivetrain.turnTo.runSequentially();

    }

    private void driveToDistance(double inches, double speed){
        //this negative sign is to correct the negation
        _base.drivetrain.driveTo.goTo(-inches, speed);
        _base.drivetrain.driveTo.runSequentially();
    }


}
