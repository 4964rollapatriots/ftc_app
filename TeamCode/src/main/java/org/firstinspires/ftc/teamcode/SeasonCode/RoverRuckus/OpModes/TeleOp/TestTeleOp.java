package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.Drivetrain.Drivetrain;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * Created by pmkf2 on 12/7/2018.
 */

@TeleOp(name ="test teleop", group = "TeleOp")

public class TestTeleOp extends LinearOpMode
{
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    @Override
    public void runOpMode() throws InterruptedException {
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        //Set State of the Drivetrain


        //run teleop while opmode is active
        while (opModeIsActive())
        {
            drive(-com.qualcomm.robotcore.util.Range.clip(gamepad1.left_stick_y, -1, 1),
                    com.qualcomm.robotcore.util.Range.clip(gamepad1.right_stick_x, -1, 1));
        }
    }
    public void drive(double drivePower, double rotatePower)
    {
        frontRight.setPower(drivePower + rotatePower);
        frontLeft.setPower(drivePower - rotatePower);
        backLeft.setPower(drivePower - rotatePower);
        backRight.setPower(drivePower + rotatePower);
        telemetry.addData("FrontRight ", frontRight.getPower());
        telemetry.addData("FrontLeft", frontLeft.getPower());
        telemetry.addData("backLeft ", backLeft.getPower());
        telemetry.addData("BackRight ", backRight.getPower());
        telemetry.update();
    }

}
