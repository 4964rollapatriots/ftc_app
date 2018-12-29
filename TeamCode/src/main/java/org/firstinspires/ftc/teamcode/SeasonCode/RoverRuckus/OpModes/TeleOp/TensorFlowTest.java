package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.CustomTensorFlow;
import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.UtilGoldDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.Autonomous.JoelDoubleCrater;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;

@TeleOp (name = "Tensor Flow Test")
public class TensorFlowTest extends LinearOpMode {

    private UtilGoldDetector eye;
    private CustomTensorFlow detector;

    private boolean RUN_USING_TENSOR_FLOW = false;
    private double ACCEPTABLE_CONFIDENCE = 0.4;
    private boolean scaleModeControl= false;

    private boolean align = false;
    private boolean alignControl = false;
    public void runOpMode(){

        //eye = new UtilGoldDetector(hardwareMap);
        detector = new CustomTensorFlow(hardwareMap);
        RUN_USING_TENSOR_FLOW = true;


        waitForStart();

        detector.activate();
        while(opModeIsActive()){
            detector.refresh();
            if(gamepad1.a) {
                if (scaleModeControl) {
                    scaleModeControl = false;
                    RUN_USING_TENSOR_FLOW = !RUN_USING_TENSOR_FLOW ;
                }
            }
            else
                scaleModeControl = true;

            if(gamepad1.b)
            {
                if(alignControl) {
                    alignControl = false;
                    align = !align;
                }
            }
            if(align) {
                telemetry.addData("found block ", RUN_USING_TENSOR_FLOW);
                telemetry.update();
                if (aligned()) {
                    telemetry.addData("found block ", RUN_USING_TENSOR_FLOW);
                    //telemetry.update();
                } else {
                    telemetry.addData("DID NOT FIND ", RUN_USING_TENSOR_FLOW);
                    //telemetry.update();
                }
            }
        }
        if(detector != null)
        {
            detector.deactivate();
        }
    }

    private boolean aligned(){
        boolean aligned = false;
        if (true){
            if(detector.updatedRecognitions == null)
            {
                aligned = false;
            }
            else{
                for (int i = 0; i <= detector.updatedRecognitions.size(); i ++){
                    Recognition rec = detector.updatedRecognitions.get(i);
                    if (rec.getLabel().equals(LABEL_GOLD_MINERAL) && rec.getConfidence() > ACCEPTABLE_CONFIDENCE){
                        aligned = true;
                        break;
                    }
                }
            }

        }
        else{
//            if (eye.isAligned()){
//                aligned = true;
//            }
        }
        return aligned;


    }
}
