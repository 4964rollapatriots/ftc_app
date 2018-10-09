package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;

@TeleOp(name = "MainTeleOp(Pranal)", group = "TeleOp")
public class TeleOpMain extends LinearOpMode
{
    private Base _base = new Base();

    public void runOpMode() throws InterruptedException
    {
        _base.init(hardwareMap, this);

        waitForStart();

        _base.drivetrain.encoderOn();

        //Set State of the Drivetrain
        _base.drivetrain.setCurrState(Drivetrain.State.FORWARD_FAST);

        //run teleop while opmode is active
        while(opModeIsActive())
        {
            run();
        }

    }
    //The Actual Teleop commands
    private void run()
    {
        _base.drivetrain.run(-gamepad1.left_stick_y , gamepad1.right_stick_x, false, false);

        _base.collector.run(Math.abs(gamepad1.right_trigger) - Math.abs(gamepad1.left_trigger));




    }


}
