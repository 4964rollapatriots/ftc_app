package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Components.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.DTBaseOnly;

/**
 * Created by pmkf2 on 11/27/2018.
 */
//We will use this teleop to only test the drivetrain related components. This will be convenient
//as we won't need to always comment the rest of the robot out in Base.java when we aren't using it.
@TeleOp(name = "DTMainTeleOp", group = "TeleOp")
public class DTTeleOpMainOnly extends LinearOpMode
{
    private DTBaseOnly _base = new DTBaseOnly() ;

    double joelAngle = 0;
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
            _base.outTelemetry();

        }

    }
    //The Actual Teleop commands
    private void run()
    {
        _base.drivetrain.run(Range.clip(-gamepad1.left_stick_y, -1, 1), Range.clip(gamepad1.right_stick_y, -1, 1), false,1);
    }

}

