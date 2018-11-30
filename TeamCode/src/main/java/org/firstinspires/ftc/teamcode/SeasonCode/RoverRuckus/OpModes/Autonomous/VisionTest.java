// This autonomous is intended for an initial setup where the robot is hanging from the lander facing the crater
// The robot will detach and lower itself and then locate the position of the block using the phone
// The robot will knock the block off and then drive to our teammate's side, where it will knock their block off also
// The robot will then deposit our team marker and park in the crater in was initially facing

package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.UtilCV;
import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.UtilGoldDetector;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.DTBaseOnly;

@Autonomous(name = "Vision Test")

// the name of the class is misleading, refer to the Autonomous name
//this is the main double crater auto
public class VisionTest extends LinearOpMode {

    private DTBaseOnly _base = new DTBaseOnly();
    private UtilGoldDetector eye;

    private blockState _block;
    private boolean secondBlockFound;

    private final static double FIRST_BLOCK_TURN_ANGLE = 35;
    private final static double MIDDLE_ANGLE = 10;
    private final static double SECOND_BLOCK_ABORT_ANGLE = 233;
    private final static double MARKER_ANGLE = 180;

    private final static double TURN_SPEED = 0.4;
    private final static double DRIVING_SPEED = 0.65;

    // these are the only final values that are used multiple times
    private final static double BLOCK_DISTANCE = 30.0;
    private final static double SECOND_BLOCK_DISTANCE = 32.0;

    //Hold state of where gold block is sitting
    private enum blockState
    {
        LEFT, MIDDLE, RIGHT, UNCERTAIN
    }

    @Override
    public void runOpMode()
    {
        _block = blockState.UNCERTAIN;
        secondBlockFound = false;
        _base.init(hardwareMap, this);
        _base.imu.calibrateTo(0);
        eye = new UtilGoldDetector(hardwareMap);
        //This calibration is done before landing because the landing could "bump" the robot and change our angle

        waitForStart();

        //Gets the robot onto the field from the hanger
        // code here

        //makes sure the landing did not get our robot off course by turning to the angle that we initialized our gyroscope to
        _base.drivetrain.turnTo.goTo(0,TURN_SPEED);
        _base.drivetrain.turnTo.runSequentially();
        // method that sends information about our angles and powers
        sendTelemetry();

        // sees if the block is initially aligned in the middle, if so it does not need to turn
        if (eye.isAligned()){
            _block = blockState.MIDDLE;
        }
        // if it does not see it, it will turn by single degrees until it reaches the first angle
        // if the block is on the left, the robot will see it as it turns
        else if (_block == blockState.UNCERTAIN){
            for (int i = 0; i < FIRST_BLOCK_TURN_ANGLE; i += 4){
                _base.drivetrain.turnTo.goTo(i,TURN_SPEED);
                _base.drivetrain.turnTo.runSequentially();
                if (eye.isAligned()){
                    _block = blockState.LEFT;
                    break;
                }
            }
        }
        //if the block is not on the left, then the robot turns to the initial zero degrees
        // it then turns slightly to the right to make sure that the block is not in the middle
        // this is necessary because of the offset of our camera on the left of our robot
        else if (_block == blockState.UNCERTAIN){
            _base.drivetrain.turnTo.goTo(0,TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially();
            for (int i = 0; i < MIDDLE_ANGLE; i += 4){
                _base.drivetrain.turnTo.goTo(360-i, TURN_SPEED);
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
            _base.drivetrain.turnTo.goTo(360 - MIDDLE_ANGLE,TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially();
            for (int i = 0; i < FIRST_BLOCK_TURN_ANGLE - MIDDLE_ANGLE; i += 4){
                _base.drivetrain.turnTo.goTo(360-MIDDLE_ANGLE-i,TURN_SPEED);
                _base.drivetrain.turnTo.runSequentially();
                if (eye.isAligned()){
                    _block = blockState.RIGHT;
                    break;
                }
            }
        }


        sendTelemetry();
        telemetry.addData("block state is", _block);
        telemetry.update();

        //drive forward to knock the block off and then go back the same distance
        // this works because at this point the robot is facing the block
        _base.drivetrain.driveTo.goTo(BLOCK_DISTANCE,DRIVING_SPEED);
        _base.drivetrain.driveTo.runSequentially();
        _base.drivetrain.driveTo.goTo(-BLOCK_DISTANCE/2,DRIVING_SPEED);
        _base.drivetrain.driveTo.runSequentially();
    }

    private void sendTelemetry(){
        telemetry.addData("Angle Z: ", _base.imu.zAngle());
        telemetry.addData("Angle X: ", _base.imu.xAngle());
        telemetry.addData("Angle Y: ", _base.imu.yAngle());
        telemetry.addData("SPEED: ", _base.drivetrain.frontLeft().getPower());
        telemetry.addData("robot is lined up", eye.isAligned());
        telemetry.update();
    }

}


