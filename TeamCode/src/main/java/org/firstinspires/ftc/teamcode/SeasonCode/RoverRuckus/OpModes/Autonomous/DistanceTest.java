package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.DTBaseOnly;

/**
 * Created by Pranal Madria on 11/28/2018.
 */
@Autonomous(name = "12Inch Distance Test NO RANGE SENSOR")
//This class will be used test the robot driving 12 inches, to see whether the encoder counts to inch constant needs to be changed
public class DistanceTest extends LinearOpMode
{
    private Base _base = new Base();
    @Override
    public void runOpMode()
    {
        _base.init(hardwareMap, this);
        _base.imu.calibrateTo(0);
        _base.drivetrain.encoderStopReset();
        _base.drivetrain.encoderOn();
        //This calibration is done before landing because the landing could "bump" the robot and change our angle

        waitForStart();

        //Gets the robot onto the field from the hanger
        // code here

        //makes sure the landing did not get our robot off course by turning to the angle that we initialized our gyroscope to
        _base.outTelemetry.write("Should travel 12 inches at .5");
        _base.outTelemetry.addData("Average Enc Counts before Moving", _base.drivetrain.getAverageEncoderCounts());
        _base.outTelemetry.update();
        _base.drivetrain.turnTo.goTo(170, .1);
        _base.drivetrain.turnTo.arcSequentially(10, 3.5);
        _base.outTelemetry.addData("Front Right Encoders: ", _base.drivetrain.frontRightEncoderCount());
        _base.outTelemetry.addData("Front Left Encoders: ", _base.drivetrain.frontLeftEncoderCount());
        _base.outTelemetry.addData("Back Right Encoders: ", _base.drivetrain.backRightEncoderCount());
        _base.outTelemetry.addData("Back Left Encoders: ", _base.drivetrain.backLeftEncoderCount());
        _base.outTelemetry.addData("Average Encoders: ", _base.drivetrain.getAverageEncoderCounts());
        _base.outTelemetry.update();
        // method that sends information about our angles and powers
        //_base.outTelemetry();
    }
}
