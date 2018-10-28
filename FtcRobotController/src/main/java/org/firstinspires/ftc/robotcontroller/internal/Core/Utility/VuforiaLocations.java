package org.firstinspires.ftc.robotcontroller.internal.Core.Utility;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

public class VuforiaLocations {

    public static final String TAG = "Vuforia Positioning System";

    OpenGLMatrix lastLocation;

    VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters parameters;

    VuforiaTrackable firstPic;
    VuforiaTrackable secondPic;
    VuforiaTrackable thirdPic;
    VuforiaTrackable fourthPic;
    List<VuforiaTrackable> allTrackables;

    float mmPerInch = 25.4f;
    float mmBotWidth = mmPerInch * 18;
    float mmfieldWidth = mmPerInch * (12*12-2);

    static final int X = 0;
    static final int Y = 1;
    static final int Z = 2;

    public VuforiaLocations(HardwareMap map){
        int camaraMonitorViewID = map.appContext.getResources().getIdentifier("camaraMonitorViewID", "id", map.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(camaraMonitorViewID);
        parameters.vuforiaLicenseKey =
                "AfMlGNH/////AAABmSa3gXBnC0isoB6lbpdqh+ZpQ5x6bhx9h1x1DElrGnq+v/2YOuaFpugQxIG7LoveU7vW9MigQz/qBL7oKt0eWjouuqmFOew" +
                        "iup+NUSwmUnnsdYzF2Ofy/4Yh2Hn1es0nC2i/oOyIq/ii4aA60fRINMqkYmyvYp3g5tU+e2A76Biux6LWr/ctNRJ4KLZZeeSOV0wbJ6bbEIEtgY" +
                        "ULHwXLyev7lz+w6TgFCYS++2BcTSmJNsLSnIMuqGOXIL8+hLbX6Lt+WGPSAlVpxr8CgTF1h+jNk6/JUB9VQuK83uUcIAT61xEr3iScylPWyF3XB" +
                        "Ql5oTCsBDMhJ+a96DBa9DMs0LF/HlSBsm3I22CuvzX0w8eB";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //vuforia = ClassFactory.getInstance().createVuforia(parameters);
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

        allTrackables = new ArrayList<VuforiaTrackable>();
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

        pictures.activate();

    }

    public void Update(){
        for (VuforiaTrackable picture: allTrackables){
            // if the picture can be seen, make the last location the location seen with the use of this picture

            OpenGLMatrix botMovements = ((VuforiaTrackableDefaultListener)picture.getListener()).getUpdatedRobotLocation();

            if (botMovements != null){
                lastLocation = botMovements;
            }
        }
    }

    public String getLocation(){
        String location = null;
        if (lastLocation != null){
            location = format(lastLocation);
        }
        return location;
    }
    public double getAngle(int type){
        double angle = 0;
        if (lastLocation != null){
            if (type == VuforiaLocations.X){
                angle = Orientation.getOrientation(lastLocation,AxesReference.EXTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES).firstAngle;
            }
            else if (type == VuforiaLocations.Y){
                angle = Orientation.getOrientation(lastLocation,AxesReference.EXTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES).secondAngle;
            }
            else if (type == VuforiaLocations.Z){
                angle = Orientation.getOrientation(lastLocation,AxesReference.EXTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES).thirdAngle;
            }
        }
        return angle;
    }

    String format(OpenGLMatrix transformMatrix){

        return transformMatrix.formatAsTransform();
    }
}