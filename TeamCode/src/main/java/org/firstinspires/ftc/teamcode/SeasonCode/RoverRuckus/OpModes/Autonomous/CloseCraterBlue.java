package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.RoverVuforia;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;


@Autonomous(name= "Blue: CRATER")
public class CloseCraterBlue extends LinearOpMode
{

    public Base _base = new Base();
    //private RoverVuforia _vuforia = new RoverVuforia();

    //Hold state of where gold block is sitting
    private enum BlocKState
    {
        LEFT, MIDDLE, RIGHT, UNCERTAIN
    }

    @Override
    public void runOpMode()
    {
        BlocKState _block = BlocKState.UNCERTAIN;

        _base.init(hardwareMap, this);
        _base.imu.calibrateTo(90);//TWEAK THIS IDK

        waitForStart();

        _base.drivetrain.driveTo.goTo(1000, .4);
        _base.drivetrain.driveTo.runSequentially();

        switch (_block)
        {
            case LEFT:
                break;
            case RIGHT:
                break;
            case MIDDLE:
                break;
            case UNCERTAIN:
                break;
        }
        _base.drivetrain.turnTo.goTo(180, .70);
        _base.drivetrain.turnTo.runSequentially();

        _base.drivetrain.driveTo.goTo(500, .3);


        sleep(2000); //test

//        while(true) //ooooffff don't use these if possible, in this case it is necessary though
//        {
//
//        }

        _base.drivetrain.stop();
    }



}
