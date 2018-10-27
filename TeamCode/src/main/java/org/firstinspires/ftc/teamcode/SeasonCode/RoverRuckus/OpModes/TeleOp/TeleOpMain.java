package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Components.HookLift.HookLift;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;

@TeleOp(name = "MainTeleOp(Pranal)", group = "TeleOp")
public class TeleOpMain extends LinearOpMode
{
    private Base _base = new Base();

    private HookLift _lift = new HookLift();

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
            telemetry.addData("Back Left Encoder", _base.drivetrain.backLeft().getCurrentPosition());
            telemetry.addData("Front Left Encoder", _base.drivetrain.frontLeft().getCurrentPosition());
            telemetry.addData("Front Right Encoder", _base.drivetrain.frontRight().getCurrentPosition());
            telemetry.addData("Back Right Encoder", _base.drivetrain.backRight().getCurrentPosition());
            _base.drivetrain.imu.setAngle();
            telemetry.addData("x angle", _base.drivetrain.imu.xAngle());
            telemetry.addData("y angle", _base.drivetrain.imu.yAngle());
            telemetry.addData("z angle", _base.drivetrain.imu.zAngle());
            telemetry.update();
        }

    }
    //The Actual Teleop commands
    private void run()
    {
        _base.drivetrain.run(-gamepad1.left_stick_y , gamepad1.right_stick_x, false, false);

        //_base.collector.run(Math.abs(gamepad1.right_trigger) - Math.abs(gamepad1.left_trigger));




    }


}
