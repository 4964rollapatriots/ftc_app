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

@Autonomous(name = "Meet Double Crater")

// the name of the class is misleading, refer to the Autonomous name
//this is the main double crater auto
public class MeetDoubleCrater extends LinearOpMode {

    private Base _base = new Base();
    private UtilGoldDetector eye;
    private CustomTensorFlow detector;


    private boolean RUN_USING_TENSOR_FLOW = true; // if not, then running with open cv is assumed

    private blockState _block;

    private final static double FAR_PARTICLE_ANGLE = 16;
    private final static double MIDDLE_ANGLE = 10;
    private final static double SECOND_BLOCK_ABORT_ANGLE = 315;
    private final static double MARKER_ANGLE = 184;

    private final static double TURN_INCREMENT = 5;

    private final static double TURN_SPEED = 0.45;
    private final static double BLOCK_TURN_SPEED = 0.55;
    private final static double DRIVING_SPEED = 0.63;
    private final static double DRIVING_SPEED_CRATER = .88;
    private final static double DRIVING_SPEED_BLOCK = .53;

    // these are the only final values that are used multiple times
    private double BLOCK_DISTANCE = 30.5;
    private final static double SECOND_BLOCK_DISTANCE = 23.0;

    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final double ACCEPTABLE_CONFIDENCE = 0.40;

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
        _base.imu.calibrateTo(0);
        eye = new UtilGoldDetector(hardwareMap);
        detector = new CustomTensorFlow(hardwareMap);
        RUN_USING_TENSOR_FLOW = true;
        //This calibration is done before landing because the landing could "bump" the robot and change our angle
        _base.outTelemetry.write("All Systems Go");
        _base.outTelemetry.update();

        waitForStart();


        //Gets the robot onto the field from the hanger

//
//        _base.latchSystem.extendHook();
//        _base.latchSystem.lowerRobot();
//
//        this.sleep(2000);


        //makes sure the landing did not get our robot off course by turning to the angle that we initialized our gyroscope to
        _base.drivetrain.turnTo.goTo(0,BLOCK_TURN_SPEED);
        _base.drivetrain.turnTo.blockRunSequentially();

        _base.deliver.markerDelivery.setPower(-.05);
        //drives forward to avoid hitting the lander while turning
        _base.drivetrain.driveTo.goTo(7,DRIVING_SPEED/2);
        _base.drivetrain.driveTo.runSequentially();
        _base.drivetrain.turnTo.goTo(335,BLOCK_TURN_SPEED);
        _base.drivetrain.turnTo.blockRunSequentially();


        // sees if the block is initially aligned in the middle, if so it does not need to turn
        RUN_USING_TENSOR_FLOW = true;

        // method that sends information about our angles and powers
        sendTelemetry();

        //turns to the far right in preparation for panning across the particles from right to left
        if(_block == blockState.UNCERTAIN)
        {
            this.sleep(400);
            if (aligned()) {
                _block = blockState.RIGHT;
                telemetry.addData("FOUND IN RIGHT", "");
                telemetry.update();
            }
            // pans across the particles until it either sees the block or reaches 348 degrees
            // 348 degrees should be past the far right particle
        }
        if (_block == blockState.UNCERTAIN){
            for  (double i = 332; i < 339; i += TURN_INCREMENT - 1){
                telemetry.addData("Searching for right block!" , "");
                telemetry.update();
                _base.drivetrain.turnTo.goTo(i,BLOCK_TURN_SPEED);
                _base.drivetrain.turnTo.blockRunSequentially();
                if (aligned()){
                    _block = blockState.RIGHT;
                    break;
                }
            }
        }
        // if it is not in the middle, the robot turns until it sees the left block or reaches 18 degrees

        //if the block is still not found, the block is on the left
        // the robot turns until it reaches 17 degrees and then pans until it sees the block
        if (_block == blockState.UNCERTAIN){
            _base.drivetrain.turnTo.goTo( 16, BLOCK_TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially();
            telemetry.addData("GO to Left,", "");
            telemetry.update();
            for (double i = FAR_PARTICLE_ANGLE; i < 35; i += TURN_INCREMENT){
                _base.drivetrain.turnTo.goTo(i,BLOCK_TURN_SPEED);
                _base.drivetrain.turnTo.blockRunSequentially();
                if (aligned()){
                    _block = blockState.LEFT;
                    break;
                }
            }
        }

        if (_block == blockState.UNCERTAIN){
            _block = blockState.MIDDLE;
            _base.drivetrain.turnTo.goTo(2, BLOCK_TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially();
        }

        //this turns a small amount to account for the offset of our phone on the left side of our robot
        turnToAlign();


        sendTelemetry();
        telemetry.addData("block state is", _block);
        telemetry.update();

        //drive forward to knock the block off and then go back the same distance
        // this works because at this point the robot is facing the block
        if (_block == blockState.MIDDLE){
            BLOCK_DISTANCE -= 9.5;
        }
        _base.drivetrain.driveTo.goTo(BLOCK_DISTANCE,DRIVING_SPEED_BLOCK);
        _base.drivetrain.driveTo.runSequentially();
        _base.drivetrain.driveTo.goTo(-(BLOCK_DISTANCE-4),DRIVING_SPEED_BLOCK);
        _base.drivetrain.driveTo.runSequentially();

        // the robot is at a common spot, but a different angle based on where the block was
        // since the turnTo class uses a gyroscope, turning to 60 gives a common angle also
        if(_block == blockState.LEFT)
        {
            _base.drivetrain.turnTo.goTo(61, TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially();
        }
        else if(_block == blockState.RIGHT)
        {
            _base.drivetrain.turnTo.goTo(57.5, TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially();
        }
        {
            _base.drivetrain.turnTo.goTo(59, TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially();
        }


        // drives between the lander and the far left particle so the path is clear to our teammate's side
        _base.drivetrain.driveTo.goTo(41, DRIVING_SPEED);
        _base.drivetrain.driveTo.runSequentially();

        //turn to drive in between particle on teammate's side and wall
        _base.drivetrain.turnTo.goTo(122.5, TURN_SPEED);
        _base.drivetrain.turnTo.runSequentially();

        //drives between the particle on teammate's side and wall
        _base.drivetrain.driveTo.goTo(11, DRIVING_SPEED);
        _base.drivetrain.driveTo.runSequentially();



        // turns in preparation for moving towards the deposit zone
        _base.drivetrain.turnTo.goTo(129, TURN_SPEED);
        _base.drivetrain.turnTo.runSequentially();

        //drives to the deposit zone
        _base.drivetrain.driveTo.goTo(24, DRIVING_SPEED);
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

        telemetry.addData("ARC TURNING" ," NOW");
        telemetry.update();

        _base.drivetrain.turnTo.goTo(174, TURN_SPEED);
        _base.drivetrain.turnTo.arcSequentially(2.0);


        // deposits the marker
        _base.deliver.deliverMarker();
        // gives time for the marker to slide off
        try{
            Thread.sleep(400);}
        catch(Exception ex){ex.printStackTrace();}
        _base.deliver.raiseMarker();
        try{
            Thread.sleep(200);}
        catch(Exception ex){ex.printStackTrace();}
        //raises the delivery system
        _base.deliver.stop();

        _base.drivetrain.driveTo.goTo(-6, DRIVING_SPEED);
        _base.drivetrain.driveTo.runSequentially();



//        _base.drivetrain.turnTo.goTo(176, TURN_SPEED);
//        _base.drivetrain.turnTo.arcSequentially(1.5);
//        _base.drivetrain.driveTo.goTo(3,DRIVING_SPEED - .2);
//        _base.drivetrain.driveTo.runSequentially();
//

        // turn to face the second group of particles
        _base.drivetrain.turnTo.goTo(229, BLOCK_TURN_SPEED);
        _base.drivetrain.turnTo.runSequentially();
        _base.drivetrain.driveTo.goTo(3, DRIVING_SPEED);
        _base.drivetrain.driveTo.runSequentially();


        // turns by degrees clockwise until the robot sees the second block or it is passes a certain angle
        for (double i = 229; i < SECOND_BLOCK_ABORT_ANGLE; i += TURN_INCREMENT + 1){
            _base.drivetrain.turnTo.goTo(i, BLOCK_TURN_SPEED);
            _base.drivetrain.turnTo.blockRunSequentially();
            if (aligned()){
                break;
            }
        }

        // since the phone is on the left of our robot, we turn a small amount so the robot's center is aligned with the block
        turnToAlign(15);


        // knock the second block off and comes back
        _base.drivetrain.driveTo.goTo(SECOND_BLOCK_DISTANCE,DRIVING_SPEED);
        _base.drivetrain.driveTo.runSequentially();
        _base.drivetrain.driveTo.goTo(-(SECOND_BLOCK_DISTANCE + 7),DRIVING_SPEED);
        _base.drivetrain.driveTo.runSequentially();

        // turn back to face the crater
        _base.drivetrain.turnTo.goTo(360-45, TURN_SPEED);
        _base.drivetrain.turnTo.runSequentially();

        // drive to the crater
        _base.drivetrain.driveTo.goTo(90,DRIVING_SPEED_CRATER);
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
        if(_block == blockState.LEFT)
        {
            _base.drivetrain.turnTo.goTo(_base.imu.zAngle() + 18, TURN_SPEED);
            _base.drivetrain.turnTo.blockRunSequentially();
            telemetry.addData("Tur to Align LEFT,", "");
            telemetry.update();
        }
        else if (_block == blockState.RIGHT)
        {
            _base.drivetrain.turnTo.goTo(_base.imu.zAngle() + 2, TURN_SPEED);
            _base.drivetrain.turnTo.blockRunSequentially();
            telemetry.addData("Tur to Align RIGHT,", "");
            telemetry.update();
        }
        else {
            _base.drivetrain.turnTo.goTo(_base.imu.zAngle() + 12, TURN_SPEED);
            _base.drivetrain.turnTo.blockRunSequentially();
            telemetry.addData("Tur to Align MIDDLE,", "");
            telemetry.update();
        }
    }
    private void turnToAlign(double ANGLE_ADDITION){
        _base.drivetrain.turnTo.goTo(_base.imu.zAngle() + ANGLE_ADDITION, TURN_SPEED);
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


