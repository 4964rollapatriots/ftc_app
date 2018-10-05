package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp.PrototypeRobot;

@Autonomous (name = "Test Auto")

public class TestAuto extends LinearOpMode {

    PrototypeRobot robot = new PrototypeRobot("jim");



    @Override
    public void runOpMode(){
        robot.init();
        telemetry.update();
        waitForStart();

        moveForward();

    }

    private void moveForward(){
        robot.drivetrain.driveToDistance(200);
    }
}
