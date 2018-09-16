package org.firstinspires.ftc.robotcontroller.internal.Core.Utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Computer on 9/15/2018.
 */

@TeleOp(name="Vuforia Locations")



public class RoverVuforia extends LinearOpMode{

    public static final String TAG = "Vuforia Positioning System";

    OpenGLMatrix lastLocation;

    VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters parameters;

    VuforiaTrackable firstPic;
    VuforiaTrackable secondPic;
    VuforiaTrackable thirdPic;
    VuforiaTrackable fourthPic;

    float mmPerInch = 25.4f;
    float mmBotWidth = mmPerInch * 18;
    float mmfieldWidth = mmPerInch * (12*12-2);

    @Override public void runOpMode(){

        //defines the parameters for the vuforia

        int camaraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("camaraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(camaraMonitorViewID);
        parameters.vuforiaLicenseKey = " the key ";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        // loads the image data from the assets folder

        VuforiaTrackables pictures = this.vuforia.loadTrackablesFromAsset("RoverRuckus");

        // sets the pictures to the data

        firstPic = pictures.get(0);
        firstPic.setName("firstPic");
        secondPic = pictures.get(1);
        secondPic.setName("secondPic");
        thirdPic = pictures.get(2);
        thirdPic.setName("thirdPic");
        fourthPic = pictures.get(3);
        fourthPic.setName("fourthPic");

        // a list of all pictures

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(pictures);

        //  sets each picture to its location on the field

        OpenGLMatrix firstLocation = OpenGLMatrix.translation(-mmfieldWidth/2,0,0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC,AxesOrder.XZX,AngleUnit.DEGREES,90,90,0));
        firstPic.setLocation(firstLocation);
        RobotLog.ii(TAG,"first pic=%s", format(firstLocation));

        OpenGLMatrix secondLocation = OpenGLMatrix.translation(0,mmfieldWidth/2,0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC,AxesOrder.XZX,AngleUnit.DEGREES,90,0,0));
        secondPic.setLocation(secondLocation);
        RobotLog.ii(TAG,"second pic=%s", format(secondLocation));

        OpenGLMatrix thirdLocation = OpenGLMatrix.translation(mmfieldWidth/2,0,0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC,AxesOrder.XZX,AngleUnit.DEGREES,90,-90,0));
        thirdPic.setLocation(thirdLocation);
        RobotLog.ii(TAG,"third pic=%s", format(thirdLocation));

        OpenGLMatrix fourthLocation = OpenGLMatrix.translation(0,-mmfieldWidth/2,0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC,AxesOrder.XZX,AngleUnit.DEGREES,-90,0,0));
        fourthPic.setLocation(fourthLocation);
        RobotLog.ii(TAG,"fourth pic=%s", format(fourthLocation));

        // defines where the phone is on the robot

        OpenGLMatrix phoneLocation = OpenGLMatrix.translation(mmBotWidth/2,0,0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC,AxesOrder.YZY,AngleUnit.DEGREES,0,0,0));
        RobotLog.ii(TAG,"phone loc=%s", format(phoneLocation));

        // lets the pictures know where the phone is relative to the robot

        ((VuforiaTrackableDefaultListener)firstPic.getListener()).setPhoneInformation(phoneLocation,parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)secondPic.getListener()).setPhoneInformation(phoneLocation,parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)thirdPic.getListener()).setPhoneInformation(phoneLocation,parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)fourthPic.getListener()).setPhoneInformation(phoneLocation,parameters.cameraDirection);

        waitForStart();

        pictures.activate();

        while (opModeIsActive()){
            for (VuforiaTrackable picture: allTrackables){
                telemetry.addData(picture.getName(),((VuforiaTrackableDefaultListener)picture.getListener()).isVisible() ? "Visible" : "Not Visible");

                // if the picture can be seen, make the lsat location the location seen with the use of this picture

                OpenGLMatrix botMovements = ((VuforiaTrackableDefaultListener)picture.getListener()).getUpdatedRobotLocation();

                if (botMovements != null){
                    lastLocation = botMovements;
                }
            }
            if (lastLocation != null){
                telemetry.addData ("position ", format(lastLocation));
                double angle = Orientation.getOrientation(lastLocation,AxesReference.EXTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES).thirdAngle;
                telemetry.addData("angle might be ", angle);
            }

            else{
                telemetry.addData("position ", "unknown");
            }
            telemetry.update();
        }

    }

    String format(OpenGLMatrix transformMatrix){
        return transformMatrix.formatAsTransform();
    }
}
