package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.Drivetrain.Drivetrain;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * Created by pmkf2 on 12/7/2018.
 */

public class TestTeleOp extends LinearOpMode
{
    DcMotor winch;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    DcMotor pulleys;
    private DcMotor collector;
    private DcMotor extendCollector;
    @Override
    public void runOpMode() throws InterruptedException {
        winch = hardwareMap.dcMotor.get("winch");
        backLeft = hardwareMap.dcMotor.get("backLeft");

        waitForStart();

        //Set State of the Drivetrain


        //run teleop while opmode is active
        while (opModeIsActive())
        {

        }
    }

}
