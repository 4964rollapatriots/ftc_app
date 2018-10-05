package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Pranal's FourWheelTeleop", group = "TeleOp")
public class FourWheelTestTeleop extends OpMode
{
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor collector;
    @Override
    public void init()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight = hardwareMap.dcMotor.get("backRight");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //collector = hardwareMap.dcMotor.get("collector");
        //collector.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void start()
    {
        super.start();
    }

    @Override
    public void loop()
    {
        if (gamepad1.a){
            //collector.setPower(1);
        }
        else if (gamepad1.b){
            //collector.setPower(-1);
        }
        else{
            //collector.setPower(0);
        }
        frontLeft.setPower(Range.clip(gamepad1.left_stick_y,-1,1) - Range.clip(gamepad1.right_stick_x,-1,1));
        backLeft.setPower(Range.clip(gamepad1.left_stick_y,-1,1) - Range.clip(gamepad1.right_stick_x,-1,1));
        frontRight.setPower(Range.clip(gamepad1.left_stick_y,-1,1) + Range.clip(gamepad1.right_stick_x,-1,1));
        backRight.setPower(Range.clip(gamepad1.left_stick_y,-1,1) + Range.clip(gamepad1.right_stick_x,-1,1));

    }

}
