// This autonomous is intended for an initial setup where the robot is hanging from the lander facing the crater
// The robot will detach and lower itself and then locate the position of the block using the phone
// The robot will knock the block off and then drive to our teammate's side, where it will knock their block off also
// The robot will then deposit our team marker and park in the crater in was initially facing

package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.CustomTensorFlow;
import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.UtilGoldDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Components.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.DTBaseOnly;

@Autonomous(name = "Joel Double Crater")

// the name of the class is misleading, refer to the Autonomous name
//this is the main double crater auto
public class JoelDoubleCrater extends LinearOpMode {

    private DTBaseOnly _base = new DTBaseOnly();
    private UtilGoldDetector eye;
    private CustomTensorFlow detector;


    private boolean RUN_USING_TENSOR_FLOW; // if not, then running with open cv is assumed

    private blockState _block;
    private boolean secondBlockFound;

    private final static double FIRST_BLOCK_TURN_ANGLE = 35;
    private final static double MIDDLE_ANGLE = 10;
    private final static double SECOND_BLOCK_ABORT_ANGLE = 315;
    private final static double MARKER_ANGLE = 184;

    private final static double TURN_INCREMENT = 3;

    private final static double TURN_SPEED = 0.45;
    private final static double BLOCK_TURN_SPEED = 0.55;
    private final static double DRIVING_SPEED = 0.63;
    private final static double DRIVING_SPEED_CRATER = .88;
    private final static double DRIVING_SPEED_BLOCK = .53;

    // these are the only final values that are used multiple times
    private final static double BLOCK_DISTANCE = 30.5;
    private final static double SECOND_BLOCK_DISTANCE = 28.0;

    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final double ACCEPTABLE_CONFIDENCE = 0.50;

    //Hold state of where gold block is sitting
    private enum blockState
    {
        LEFT, MIDDLE, RIGHT, UNCERTAIN
    }

    @Override
    public void runOpMode()
    {
        _base.init(hardwareMap, this);
        _base.outTelemetry.write("Initializing - DO NOT START UNTIL NEXT MESSAGE");
        _base.outTelemetry.update();
        _block = blockState.UNCERTAIN;
        secondBlockFound = false;
        _base.imu.calibrateTo(0);
        eye = new UtilGoldDetector(hardwareMap);
        detector = new CustomTensorFlow(hardwareMap);
        RUN_USING_TENSOR_FLOW = false;
        //This calibration is done before landing because the landing could "bump" the robot and change our angle
        _base.outTelemetry.write("All Systems Go");
        _base.outTelemetry.update();

        waitForStart();

        //Gets the robot onto the field from the hanger
        // code here

        //makes sure the landing did not get our robot off course by turning to the angle that we initialized our gyroscope to
        _base.drivetrain.turnTo.goTo(0,TURN_SPEED);
        _base.drivetrain.turnTo.blockRunSequentially();
        // method that sends information about our angles and powers
        sendTelemetry();

        // sees if the block is initially aligned in the middle, if so it does not need to turn
        if (aligned()){
            _block = blockState.MIDDLE;
            telemetry.addData("FOUND IN MIDDLE" , "");
        }
        // if it does not see it, it will turn by single degrees until it reaches the first angle
        // if the block is on the left, the robot will see it as it turns
        if (_block == blockState.UNCERTAIN){
            for (int i = 20; i < FIRST_BLOCK_TURN_ANGLE; i += TURN_INCREMENT){
                _base.drivetrain.turnTo.goTo(i,BLOCK_TURN_SPEED);
                _base.drivetrain.turnTo.blockRunSequentially();
                if (aligned()){
                    _block = blockState.LEFT;
                    // turn some to account for the phone being on the left of our robot
                    turnToAlign();
                    telemetry.addData("FOUND IN LEFT" , "");
                    telemetry.update();
                    break;
                }
            }
        }
        //if the block is not on the left, then the robot turns to the initial zero degrees
        // it then turns slightly to the right to make sure that the block is not in the middle
        // this is necessary because of the offset of our camera on the left of our robot
        if (_block == blockState.UNCERTAIN){
            _base.drivetrain.turnTo.goTo(7,TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially();
            for (int i = 20; i < MIDDLE_ANGLE; i += TURN_INCREMENT){
                _base.drivetrain.turnTo.goTo(360-i, BLOCK_TURN_SPEED);
                _base.drivetrain.turnTo.blockRunSequentially();
                if (aligned()){
                    _block = blockState.MIDDLE;
                    // turn some to account for the phone being on the left of our robot
                    turnToAlign();
                    telemetry.addData("FOUND IN RIGHT" , "");
                    telemetry.update();
                    break;
                }
            }
        }
        // if and only if the robot has not yet found the ball,
        // it turns to the right by small increments until it reaches its target or sees the block
        if (_block == blockState.UNCERTAIN){
            _base.drivetrain.turnTo.goTo(360 - MIDDLE_ANGLE,TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially();
            for (int i = 0; i < FIRST_BLOCK_TURN_ANGLE - MIDDLE_ANGLE; i += TURN_INCREMENT){
                _base.drivetrain.turnTo.goTo(360-MIDDLE_ANGLE-i,BLOCK_TURN_SPEED);
                _base.drivetrain.turnTo.blockRunSequentially();
                if (aligned()){
                    _block = blockState.RIGHT;
                    turnToAlign();
                    break;
                }
            }
        }


        sendTelemetry();
        telemetry.addData("block state is", _block);
        telemetry.update();

        //drive forward to knock the block off and then go back the same distance
        // this works because at this point the robot is facing the block
        _base.drivetrain.driveTo.goTo(BLOCK_DISTANCE,DRIVING_SPEED_BLOCK);
        _base.drivetrain.driveTo.runSequentially();
        _base.drivetrain.driveTo.goTo(-(BLOCK_DISTANCE-5),DRIVING_SPEED_BLOCK);
        _base.drivetrain.driveTo.runSequentially();

        // the robot is at a common spot, but a different angle based on where the block was
        // since the turnTo class uses a gyroscope, turning to 60 gives a common angle also
        _base.drivetrain.turnTo.goTo(60,TURN_SPEED);
        _base.drivetrain.turnTo.runSequentially();


        // drives between the lander and the far left particle so the path is clear to our teammate's side
        _base.drivetrain.driveTo.goTo(40, DRIVING_SPEED);
        _base.drivetrain.driveTo.runSequentially();

        //turn to drive in between particle on teammate's side and wall
        _base.drivetrain.turnTo.goTo(122, TURN_SPEED);
        _base.drivetrain.turnTo.runSequentially();

        //drives between the particle on teammate's side and wall
        _base.drivetrain.driveTo.goTo(20, DRIVING_SPEED);
        _base.drivetrain.driveTo.runSequentially();



        // turns in preparation for moving towards the deposit zone
        _base.drivetrain.turnTo.goTo(129, TURN_SPEED);
        _base.drivetrain.turnTo.runSequentially();

        //drives to the deposit zone
        _base.drivetrain.driveTo.goTo(36, DRIVING_SPEED);
        _base.drivetrain.driveTo.runSequentially();


//
//        // turn a little amount so the robot does not hit the wall
//        _base.drivetrain.turnTo.goTo(MARKER_ANGLE-7, TURN_SPEED *1.3);
//        _base.drivetrain.turnTo.runSequentially();
//
//        //drives a bit so the robot does not get stuck on the wall
//        _base.drivetrain.driveTo.goTo(7, DRIVING_SPEED);
//        _base.drivetrain.driveTo.runSequentially();
//
//
//        // turn the robot to deposit the marker
//        _base.drivetrain.turnTo.goTo(MARKER_ANGLE, TURN_SPEED * 1.3);
//        _base.drivetrain.turnTo.runSequentially();

        _base.drivetrain.turnTo.goTo(180, TURN_SPEED);
        _base.drivetrain.turnTo.arcSequentially(1.8);


        // deposits the marker
        _base.deliver.deliverMarker();
        // gives time for the marker to slide off
        try{
            Thread.sleep(600);}
        catch(Exception ex){ex.printStackTrace();}

        //raises the delivery system
        _base.deliver.raiseMarker();

        // turn to face the second group of particles
        _base.drivetrain.turnTo.goTo(180, TURN_SPEED);
        _base.drivetrain.turnTo.runSequentially();


        // turns by single degrees clockwise until the robot sees the second block or it is passes a certain angle
        for (double i = 15; i < SECOND_BLOCK_ABORT_ANGLE; i += TURN_INCREMENT / 2){
            _base.drivetrain.turnTo.goTo(_base.imu.zAngle() + i, TURN_SPEED);
            _base.drivetrain.turnTo.blockRunSequentially();
            if (aligned()){
                // since the phone is on the left of our robot, we turn a small amount so the robot's center is aligned with the block
                turnToAlign();
                break;
            }
        }


        // knock the second block off and comes back
        _base.drivetrain.driveTo.goTo(SECOND_BLOCK_DISTANCE,DRIVING_SPEED);
        _base.drivetrain.driveTo.runSequentially();
        _base.drivetrain.driveTo.goTo(-(SECOND_BLOCK_DISTANCE + 5),DRIVING_SPEED);
        _base.drivetrain.driveTo.runSequentially();

        // turn back to face the crater
        _base.drivetrain.turnTo.goTo(360-45, TURN_SPEED);
        _base.drivetrain.turnTo.runSequentially();

        // drive to the crater
        _base.drivetrain.driveTo.goTo(80,DRIVING_SPEED_CRATER);
        _base.drivetrain.driveTo.runSequentially();

        //we are done, so stop the robot
        _base.drivetrain.stop();
        eye.stop();
        detector.deactivate();


    }

    private void sendTelemetry(){
        telemetry.addData("Angle Z: ", _base.imu.zAngle());
        telemetry.addData("Angle X: ", _base.imu.xAngle());
        telemetry.addData("Angle Y: ", _base.imu.yAngle());
        telemetry.addData("SPEED: ", _base.drivetrain.frontLeft().getPower());
        telemetry.addData("robot is lined up", aligned());
        telemetry.update();
    }

    private void turnToAlign(){
        _base.drivetrain.turnTo.goTo(_base.imu.zAngle() + 11, TURN_SPEED);
        _base.drivetrain.turnTo.blockRunSequentially();
    }

    private boolean aligned(){
        boolean aligned = false;
        if (RUN_USING_TENSOR_FLOW){
            detector.refresh();
            if(detector.updatedRecognitions == null)
            {
                aligned = false;
            }
            else{
                for (int i = 0; i < detector.updatedRecognitions.size(); i ++){
                    Recognition rec = detector.updatedRecognitions.get(i);
                    if (rec.getLabel().equals(LABEL_GOLD_MINERAL) && rec.getConfidence() > ACCEPTABLE_CONFIDENCE){
                        aligned = true;
                        break;
                    }
                }
            }


        }
        else{
            if (eye.isAligned()){
                aligned = true;
            }
        }
        return aligned;


    }

}


