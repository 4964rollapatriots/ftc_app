    package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;

@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class TeleOpMain extends LinearOpMode
{
    //create an instance of our base which contains all of the components of the robot
    private Base _base = new Base();

    //Constants for manipulating the collector's lift power
    private double EXTEND_LIFT_POW = 1;
    private double RETRACT_LIFT_POW = -.90;

    public void runOpMode() throws InterruptedException
    {
        _base.init(hardwareMap, this);
        waitForStart();
        _base.drivetrain.encoderOn();

        //Set State of the Drivetrain
        _base.drivetrain.setCurrState(Drivetrain.State.FORWARD_FAST);

        _base.drivetrain.encoderStopReset();
        _base.drivetrain.encoderOn();
        _base.drivetrain.encoderOff();
        _base.drivetrain.imu.calibrateTo(0);


        //run teleop while opmode is active
        while(opModeIsActive())
        {
            run();

            /*
            TELEMETRY CONFIG
             */
            _base.outTelemetry();



            /*
            ANGLE TESTING BELOW
             */
            _base.drivetrain.imu.setAngle();
        }
    }
    //The Actual Teleop commands
    private void run()
    {
        /*----------------------------------- STANDARD DRIVING ----------------------------------*/
        //drive the robot using gamepad1 joysticks, standard six wheel movement
        _base.drivetrain.run(-gamepad1.left_stick_y , gamepad1.right_stick_x, false, false);

        /*------------------------------------ MARKER DELIVERY --------------------------------*/
        //this is to be used just in case the marker delivery system needs to be used
        if(gamepad1.dpad_up)
            _base.deliver.raiseMarker();
        if(gamepad1.dpad_down)
            _base.deliver.deliverMarker();

        /*---------------------------- HOOK EXTENSION/LIFT ROBOT --------------------------------*/
        if(gamepad2.a)
            _base.latchSystem.extendHook();
        if(gamepad2.b)
            _base.latchSystem.retractHook();
        if(gamepad2.x)
            _base.latchSystem.liftRobot();
        if (gamepad2.y)
            _base.latchSystem.lowerRobot();

        /* -------------- COLLECTING SYSTEM ---------------------*/

        //Non-Precision Based Extension/Retraction
        if(gamepad2.right_bumper)
            _base.collector.powerLift(EXTEND_LIFT_POW);
        if(gamepad2.left_bumper)
            _base.collector.powerLift(RETRACT_LIFT_POW);

        //Precision Based Extension/Retraction
        if(gamepad2.left_stick_y > .15 || gamepad2.left_stick_y < -.15)
        {
            if(gamepad2.left_stick_y > .85)
            {
                _base.collector.powerLift(EXTEND_LIFT_POW);
            }
            else if(gamepad2.left_stick_y < -.85)
            {
                _base.collector.powerLift(-1);
            }
            else
            {
                _base.collector.powerLift(gamepad2.left_stick_y);
            }
        }


        //Collect particles using foam wheel :)
        if(gamepad2.right_trigger > .2)
            if(gamepad2.right_trigger > .65)
                _base.collector.runCollector(1);
            else
                _base.collector.runCollector(gamepad2.right_trigger);
        if(gamepad2.left_trigger > .2)
            if(gamepad2.left_trigger > .65)
                _base.collector.runCollector(-1);
            else
                _base.collector.runCollector(-gamepad2.left_trigger);

        /*----------------Tilt C-Channel Holding Lifts---------------*/
        if(gamepad2.right_stick_y > .2)
        {
            if(gamepad2.right_stick_y > .85)
                _base.tiltChannel.tiltByPower(1);
            else
                _base.tiltChannel.tiltByPower(gamepad2.right_stick_y);
        }
        else if(gamepad2.right_stick_y < -.2)
        {
            if(gamepad2.right_stick_y < -.85)
                _base.tiltChannel.tiltByPower(-1);
            else
                _base.tiltChannel.tiltByPower(gamepad2.right_stick_y);
        }




    }

}
