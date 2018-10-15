package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp.PrototypeRobot;

@Autonomous (name = "Test Auto")

public class TestAuto extends LinearOpMode {

    Base robot = new Base();
    final static double WHEEL_DIAMETER = 10;
    final static double ENCODERS_PER_ROTATION = 1440;
    final static double  CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;


    @Override
    public void runOpMode(){
        robot.init(hardwareMap, this);
        waitForStart();

        drive(12);
        turnTo(90);
        drive(21);
        sleep(2000);
        drive(-30.0);


    }

    private void drive(double cm){
        double rotations = cm/CIRCUMFERENCE;
        int encoders = (int)(rotations * ENCODERS_PER_ROTATION);
        robot.drivetrain.testDriveForEncoders(encoders);
    }

    private void turnTo(double angle){
        robot.drivetrain.testTurnTo(angle);
    }

}
