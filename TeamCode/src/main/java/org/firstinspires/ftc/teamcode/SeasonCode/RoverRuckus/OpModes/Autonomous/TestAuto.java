package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp.PrototypeRobot;

@Autonomous (name = "Test Auto")

public class TestAuto extends LinearOpMode {

    Base robot = new Base();
    final static double WHEEL_DIAMETER = 4;
    final static
    double ENCODERS_PER_ROTATION = 1440;
    final static double  CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;


    @Override
    public void runOpMode(){
        robot.init(hardwareMap, this);
        waitForStart();
        turnTo(90.0);
        telemetry.update();


    }

    private void drive(double inches){
        double rotations = inches/CIRCUMFERENCE;
        int encoders = (int)(rotations * ENCODERS_PER_ROTATION);
        robot.drivetrain.testDriveForEncoders(encoders);
        telemetry.addData("encoders", encoders);
        telemetry.update();
    }

    private void turnTo(double desiredAngle){
        robot.drivetrain.testTurnTo(desiredAngle);
    }

}
