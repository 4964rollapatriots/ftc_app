// This autonomous is intended for an initial setup where the robot is hanging from the lander facing the crater
// The robot will detach and lower itself and then locate the position of the block using the phone
// The robot will knock the block off and then drive to our teammate's side, where it will knock their block off also
// The robot will then deposit our team marker and park in the crater in was initially facing

package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.CustomTensorFlow;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.DTBaseOnly;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;

@Autonomous(name = "Vision Test")

// the name of the class is misleading, refer to the Autonomous name
//this is the main double crater auto
public class VisionTest extends LinearOpMode {

    private DTBaseOnly _base = new DTBaseOnly();
    private CustomTensorFlow eye;

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
        eye = new CustomTensorFlow(hardwareMap);
        //This calibration is done before landing because the landing could "bump" the robot and change our angle

        waitForStart();
        eye.activate();

        //Gets the robot onto the field from the hanger
        // code here

        //makes sure the landing did not get our robot off course by turning to the angle that we initialized our gyroscope to
        // method that sends information about our angles and powers

        // sees if the block is initially aligned in the middle, if so it does not need to turn

        while(opModeIsActive()) {
            if (aligned()) {
                telemetry.addData("block FOUND! ", true);
                telemetry.update();
                _block = blockState.MIDDLE;
            }
            else{
                telemetry.addData("nothing found", true);
                telemetry.update();
            }
        }
        // if it does not see it, it will turn by single degrees until it reaches the first angle
        // if the block is on the left, the robot will see it as it turns
        telemetry.addData("block state is", _block);
        telemetry.update();

        //drive forward to knock the block off and then go back the same distance
        // this works because at this point the robot is facing the block

    }

    private boolean aligned(){
        boolean aligned = false;
        if (true){
            eye.refresh();
            if(eye.recognitions == null)
            {
                telemetry.addData("Eye List Null ", true);
                telemetry.update();
                aligned = false;
            }
            else{
                telemetry.addData("Pre For Loop in Eye ", true);
                telemetry.update();

                for (int i = 0; i < eye.recognitions.size(); i ++){
                    Recognition rec = eye.recognitions.get(i);
                    telemetry.addData("Left side ", rec.getLeft());
                    telemetry.addData("UpdatedRecognition Size ", eye.recognitions.size());
                    telemetry.addData("Rec Label", rec.getLabel());
                    telemetry.addData("Rec Confidence", rec.getConfidence());
                    telemetry.update();
                    if (rec.getLabel().equals(LABEL_GOLD_MINERAL) && rec.getConfidence() > .40){
                        aligned = true;
                        break;
                    }
                }
            }

        }
        return aligned;


    }

}


