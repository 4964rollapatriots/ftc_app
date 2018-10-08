package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Four Wheel Prototype")
public class FourWheelTeleOp extends LinearOpMode
{

    private PrototypeRobot robot = new PrototypeRobot("joel");

    @Override
    public void runOpMode()
    {

        robot.init();
        telemetry.update();
        waitForStart();

        telemetry.clear();
        robot.drivetrain.setWithoutEncoders();

        while (opModeIsActive()) {
            powerDriveTrain();
            //powerCollector();
        }
    }

    private void powerDriveTrain () {
        double leftPower = gamepad1.left_stick_y;
        double rightPower = gamepad1.right_stick_y;
        robot.drivetrain.powerLeftMotors(leftPower);
        robot.drivetrain.powerRightMotors(rightPower);

    }
    private void powerCollector(){
        if (gamepad2.a){
            //robot.collector.setPower(1);
        }
        else if (gamepad2.b){
            //robot.collector.setPower(-1);
        }
        else{
            //robot.collector.setPower(0);
        }
    }
}