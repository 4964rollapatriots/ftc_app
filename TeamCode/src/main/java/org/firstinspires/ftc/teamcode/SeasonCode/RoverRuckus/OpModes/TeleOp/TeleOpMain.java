    package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;
import org.opencv.core.Range;

    @TeleOp(name = "MainTeleOp", group = "TeleOp")
public class TeleOpMain extends LinearOpMode
{
    //create an instance of our base which contains all of the components of the robot
    private Base _base = new Base();

    //Constants for manipulating the collector's lift power
    private double EXTEND_LIFT_POW = 1;
    private double RETRACT_LIFT_POW = -.90;
    private boolean releaseParticles = false;
    private boolean scaleMode = false;
    private boolean slowMode = false;
    private boolean inverted = false;
    private boolean invertedControl = false;
    private boolean scaleModeControl = false;
    private boolean slowModeControl = false;
    private boolean releaseParticlesControl = false;

    public void runOpMode() throws InterruptedException
    {
        _base.init(hardwareMap, this);
        waitForStart();

        //Set State of the Drivetrain
        _base.drivetrain.setCurrState(Drivetrain.State.FORWARD_FAST);


        //run teleop while opmode is active
        while(opModeIsActive())
        {
            run();
        }
    }
    //The Actual Teleop commands
    private void run() {
        /*----------------------------------- STANDARD DRIVING ----------------------------------*/
        //drive the robot using gamepad1 joysticks, standard six wheel movement
        if (gamepad1.right_bumper) {
            if (invertedControl) {
                invertedControl = false;
                inverted = !inverted;
            }
        } else
        {
            invertedControl = true;
        }

        if(gamepad1.left_bumper) {
            if (slowModeControl) {
                slowModeControl = false;
                slowMode = !slowMode;
            }
        }
        else
            slowModeControl = true;

        if(gamepad1.a) {
            if (scaleModeControl) {
                scaleModeControl = false;
                scaleMode = !scaleMode ;
            }
        }
        else
            scaleModeControl = true;


        _base.drivetrain.run(-com.qualcomm.robotcore.util.Range.clip(gamepad1.left_stick_y, -1, 1),
                com.qualcomm.robotcore.util.Range.clip(gamepad1.right_stick_x, -1, 1), inverted, slowMode, scaleMode);
        telemetry.addData("Gamepad Left Stick, ", -gamepad1.left_stick_y);
        telemetry.addData("Gamepad Right Stick, ", gamepad1.right_stick_y);
        /*------------------------------------ MARKER DELIVERY --------------------------------*/
        //this is to be used just in case the marker delivery system needs to be used
        if(gamepad1.dpad_up)
            _base.deliver.raiseMarker();
        else if(gamepad1.dpad_down)
            _base.deliver.deliverMarker();
        else
            _base.deliver.stop();

        /*---------------------------- HOOK EXTENSION/LIFT ROBOT --------------------------------*/
        if(gamepad2.a)
            _base.latchSystem.extendHook();
        else if(gamepad2.b)
            _base.latchSystem.retractHook();
        else if(gamepad1.x)
            _base.latchSystem.liftRobot();
        else if (gamepad1.y)
            _base.latchSystem.lowerRobot();
        else
            _base.latchSystem.stop();

        /* -------------- COLLECTING SYSTEM ---------------------*/

        //Non-Precision Based Extension/Retraction
        if(gamepad2.right_bumper)
            _base.collector.powerLift(EXTEND_LIFT_POW);
        else if(gamepad2.left_bumper)
            _base.collector.powerLift(RETRACT_LIFT_POW);

        //Precision Based Extension/Retraction
        else if(gamepad2.left_stick_y > .15 || gamepad2.left_stick_y < -.15)
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
        else
            _base.collector.powerLift(0);


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
        if(gamepad2.right_trigger < .2 && gamepad2.left_trigger < .2)
        {
            _base.collector.runCollector(0);
        }
        if (gamepad2.x)
        {
            _base.collector.releaseParticles();
        }
        else if (gamepad2.y)
        {
            _base.collector.containParticles();
        }
        else
            _base.collector.stopCRServo();


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
        else
            _base.tiltChannel.tiltByPower(0);
//    }
        _base.outTelemetry();
    }

}
