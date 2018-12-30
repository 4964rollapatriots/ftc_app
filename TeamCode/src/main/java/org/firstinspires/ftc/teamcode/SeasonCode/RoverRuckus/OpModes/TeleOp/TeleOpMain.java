    package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;
import org.opencv.core.Range;

import static java.util.logging.Logger.global;

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

        drive(-com.qualcomm.robotcore.util.Range.clip(gamepad1.left_stick_y, -1, 1),
                com.qualcomm.robotcore.util.Range.clip(gamepad1.right_stick_x, -1, 1));

        telemetry.addData("Range: ", range)
        /*------------------------------------ MARKER DELIVERY --------------------------------*/
        //this is to be used just in case the marker delivery system needs to be used

        /*---------------------------- HOOK EXTENSION/LIFT ROBOT --------------------------------*/
//        if(gamepad2.a)
//            _base.latchSystem.extendHook();
//        else if(gamepad2.b)
//            _base.latchSystem.retractHook();
//        else if(gamepad1.x)
//            _base.latchSystem.liftRobot();
//        else if (gamepad1.y)
//            _base.latchSystem.lowerRobot();
//        else
//            _base.latchSystem.stop();
//        /* -------------- COLLECTING SYSTEM ---------------------*/
//
//        //Non-Precision Based Extension/Retraction
//        if(gamepad2.right_bumper)
//            _base.collector.powerLift(EXTEND_LIFT_POW);
//        else if(gamepad2.left_bumper)
//            _base.collector.powerLift(RETRACT_LIFT_POW);
//
//        //Precision Based Extension/Retraction
//        else if(gamepad2.left_stick_y > .15 || gamepad2.left_stick_y < -.15)
//        {
//            if(gamepad2.left_stick_y > .85)
//            {
//                _base.collector.powerLift(EXTEND_LIFT_POW);
//            }
//            else if(gamepad2.left_stick_y < -.85)
//            {
//                _base.collector.powerLift(-1);
//            }
//            else
//            {
//                _base.collector.powerLift(gamepad2.left_stick_y);
//            }
//        }
//        else
//            _base.collector.powerLift(0);
//
//
//        //Collect particles using foam wheel :)
//        if(gamepad2.right_trigger > .2)
//            if(gamepad2.right_trigger > .65)
//                _base.collector.runCollector(1);
//            else
//                _base.collector.runCollector(gamepad2.right_trigger);
//        if(gamepad2.left_trigger > .2)
//            if(gamepad2.left_trigger > .65)
//                _base.collector.runCollector(-1);
//            else
//                _base.collector.runCollector(-gamepad2.left_trigger);
//        if(gamepad2.right_trigger < .2 && gamepad2.left_trigger < .2)
//        {
//            _base.collector.runCollector(0);
//        }
//        if (gamepad2.x)
//        {
//            _base.collector.releaseParticles();
//        }
//        else if (gamepad2.y)
//        {
//            _base.collector.containParticles();
//        }
//        else
//            _base.collector.stopCRServo();
//
//
//        /*----------------Tilt C-Channel Holding Lifts---------------*/
//        if(gamepad2.right_stick_y > .2)
//        {
//            if(gamepad2.right_stick_y > .85)
//                _base.tiltChannel.tiltByPower(1);
//            else
//                _base.tiltChannel.tiltByPower(gamepad2.right_stick_y);
//        }
//        else if(gamepad2.right_stick_y < -.2)
//        {
//            if(gamepad2.right_stick_y < -.85)
//                _base.tiltChannel.tiltByPower(-1);
//            else
//                _base.tiltChannel.tiltByPower(gamepad2.right_stick_y);
//        }
//        else
//            _base.tiltChannel.tiltByPower(0);
//    }
    }
    public void drive(double drivePower, double rotatePower)
    {
        _base.drivetrain.frontRight().setPower(drivePower + rotatePower);
        _base.drivetrain.frontLeft().setPower(drivePower - rotatePower);
        _base.drivetrain.backLeft().setPower(drivePower - rotatePower);
        _base.drivetrain.backRight().setPower(drivePower + rotatePower);
    }
}
