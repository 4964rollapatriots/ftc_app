// This autonomous is intended for an initial setup where the robot is hanging from the lander facing the crater
// The robot will detach and lower itself and then locate the position of the block using the phone
// The robot will knock the block off and then drive to our teammate's side, where it will knock their block off also
// The robot will then deposit our team marker and park in the crater in was initially facing

package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.UtilCV;
import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.UtilGoldDetector;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.DTBaseOnly;

//@TeleOp (name = "Vision Test Teleop")

// the name of the class is misleading, refer to the Autonomous name
//this is the main double crater auto
public class GoldAlignExampleTest extends LinearOpMode {

    private DTBaseOnly _base = new DTBaseOnly();
    private UtilGoldDetector eye;

    @Override
    public void runOpMode()
    {
        eye = new UtilGoldDetector(hardwareMap);
        //This calibration is done before landing because the landing could "bump" the robot and change our angle

        waitForStart();

        //Gets the robot onto the field from the hanger
        // code here

        // method that sends information about our angles and powers
        //sendTelemetry();
        while(opModeIsActive())
        {
            telemetry.addData("Align Size: ", eye.getAlignSize());
            telemetry.addData("Eye is Aligned: ", eye.isAligned());
            telemetry.addData("block y value ", eye.detector.goldYPos);
            telemetry.update();
            if (gamepad1.right_stick_y > .1)
            {
                eye.setAlignSize(eye.getAlignSize() + 2);
            }
            if (gamepad1.right_stick_y < -.1)
            {
                eye.setAlignSize(eye.getAlignSize() - 2);
            }

        }
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


