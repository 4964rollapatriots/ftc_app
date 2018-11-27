package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.DTBaseOnly;

/**
 * Created by pmkf2 on 11/27/2018.
 */
//We will use this teleop to only test the drivetrain related components. This will be convenient
//as we won't need to always comment the rest of the robot out in Base.java when we aren't using it.
@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class DTTeleOpMainOnly extends LinearOpMode
{
    private DTBaseOnly _base = new DTBaseOnly() ;

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

            telemetry.addData("Inches Travelled: ", _base.drivetrain.getEncoderCounts());
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



    }

}

