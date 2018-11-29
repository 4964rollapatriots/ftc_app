    package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;

@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class TeleOpMain extends LinearOpMode
{
    private Base _base = new Base();

    double joelAngle = 0;
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
//            telemetry.addData("Back Left Encoder", _base.drivetrain.backLeft().getCurrentPosition());
//            telemetry.addData("Front Left Encoder", _base.drivetrain.frontLeft().getCurrentPosition());
//            telemetry.addData("Front Right Encoder", _base.drivetrain.frontRight().getCurrentPosition());
//            telemetry.addData("Back Right Encoder", _base.drivetrain.backRight().getCurrentPosition());
//            telemetry.addData("Get  Power BACKLEFT", _base.drivetrain.backLeft().getPower());
//            telemetry.addData("Get  Power BACKRIGHT", _base.drivetrain.backRight().getPower());
//            telemetry.addData("Get  Power FRONTLEFT", _base.drivetrain.frontLeft().getPower());
//            telemetry.addData("Get  Power FRONTRIGHT", _base.drivetrain.frontRight().getPower());


            /*
            TELEMETRY CONFIG
             */
            _base.outTelemetry();



            /*
            ANGLE TESTING BELOW
             */
            _base.drivetrain.imu.setAngle();
//
            joelAngle = _base.drivetrain.imu.zAngle() + 360;
            joelAngle %= 360;


            telemetry.addData("z angle strd", _base.drivetrain.imu.zAngle());
            telemetry.addData("z Joel Angle (what we used have)", joelAngle);

            joelAngle += 360;
            telemetry.addData("z angle + 360", joelAngle);


            telemetry.addData("x angle", _base.drivetrain.imu.xAngle());
            telemetry.addData("y angle", _base.drivetrain.imu.yAngle());
            telemetry.update();
        }
    }
    //The Actual Teleop commands
    private void run()
    {
        _base.drivetrain.run(-gamepad1.left_stick_y , gamepad1.right_stick_x, false, false);

        /*------------------------------------ MARKER DELIVERY --------------------------------*/
        //this is to be used just in case the marker delivery system needs to be used
        if(gamepad2.dpad_left)
            _base.deliver.raiseMarker();
        if(gamepad2.dpad_right)
            _base.deliver.deliverMarker();

        /*------------------------------------ HOOK EXTENSION ---------------------------------*/
        if(gamepad2.a)
            _base.latchSystem.extendHook();
        if(gamepad2.b)
            _base.latchSystem.retractHook();
        if(gamepad2.x)
            _base.latchSystem.liftRobot();
        if (gamepad2.y)
            _base.latchSystem.lowerRobot();

        /* -------------- COLLECTING SYSTEM ---------------------*/
//        if(gamepad2.right_bumper)
//            _base.collector.extendLifts();
//        if(gamepad2.left_bumper)
//            _base.collector.retractLifts();
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

        /*----------------Tilt C-Channel Holding Lifts---------------*/
        if(gamepad2.right_stick_y > .2)
        {
            _base.tiltChannel.tiltUpByPower();
        }
        if(gamepad2.right_stick_y < -.2)
        {
            _base.tiltChannel.tiltDownByPower();
        }




    }

}
