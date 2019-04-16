package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;

/**
 * Created by pmkf2 on 2/27/2019.
 */
@Autonomous(name = "Team Marker Delivery Test")
public class DeliverTeamMarker extends LinearOpMode{
    private Base _base = new Base();
    @Override
    public void runOpMode()
    {
        _base.init(hardwareMap, this);
        _base.drivetrain.encoderStopReset();
        _base.drivetrain.encoderOn();
        //This calibration is done before landing because the landing could "bump" the robot and change our angle

        waitForStart();

        //Gets the robot onto the field from the hanger
        // code here

        //makes sure the landing did not get our robot off course by turning to the angle that we initialized our gyroscope to

        _base.collector.powerExtension(-1);
        try{
            Thread.sleep(750);}
        catch(Exception ex){ex.printStackTrace();}

        _base.tiltChannel.lowestTiltDownByEnc(3000);

        sleep(300);

        _base.collector.runCollector(-1);

        // gives time for the marker to slide off
        try{
            Thread.sleep(500);}
        catch(Exception ex){ex.printStackTrace();}
        _base.collector.runCollector(0);
        _base.tiltChannel.AUTOTiltToZero(3500);
        _base.collector.powerExtension(.65);
        try{
            Thread.sleep(300);}
        catch(Exception ex){ex.printStackTrace();}
        _base.collector.stop();

        // method that sends information about our angles and powers
        //_base.outTelemetry();
    }
    private void deliver(){
        _base.collector.powerExtension(-1);
        try{
            Thread.sleep(350);}
        catch(Exception ex){ex.printStackTrace();}

        _base.tiltChannel.lowestTiltDownByEnc(3000);
        _base.collector.runCollector(-.40);

        // gives time for the marker to slide off
        try{
            Thread.sleep(750);}
        catch(Exception ex){ex.printStackTrace();}
        _base.collector.powerExtension(0);
        _base.collector.runCollector(-.25);
        _base.tiltChannel.AUTOTiltToZero(3500);
        //_base.collector.powerExtension(-.65);
//        try{
//            Thread.sleep(800);}
//        catch(Exception ex){ex.printStackTrace();}
        _base.collector.powerExtension(0);
        _base.collector.stop();
    }

}


