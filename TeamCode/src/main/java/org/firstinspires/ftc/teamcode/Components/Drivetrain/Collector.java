package org.firstinspires.ftc.teamcode.Components.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotComponent;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp.PrototypeRobot;

public class Collector extends RobotComponent {

    private PrototypeRobot bot;

    private DcMotor motor;
    private HardwareMap map;

    public Collector (PrototypeRobot robot){
        bot = robot;
    }

    public void mapHardware(){
        motor = map.dcMotor.get("collector");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setPower(double power){
        motor.setPower(power);
    }


    public void stop()
    {
        motor.setPower(0);
    }
}
