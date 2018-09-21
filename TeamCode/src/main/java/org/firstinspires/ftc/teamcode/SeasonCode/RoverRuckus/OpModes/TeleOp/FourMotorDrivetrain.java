package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.HardwareMapper;

import java.util.ArrayList;

public class FourMotorDrivetrain {

    private Robot bot;
    private HardwareMap map;

    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor frontRight;

    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();

    public FourMotorDrivetrain(Robot robot){
        bot = robot;
    }

    public void mapMotors(){
        backLeft = map.dcMotor.get("backLeft");
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft = map.dcMotor.get("frontLeft");
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight = map.dcMotor.get("backRight");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight = map.dcMotor.get("frontRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setUsingEncoders(){
        for (int i = 0; i < motors.size(); i ++){
            motors.get(i).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void resetEncoders(){
        for (int i = 0; i < motors.size(); i ++){
            motors.get(i).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void setWithoutEncoders(){
        for (int i = 0; i < motors.size(); i ++){
            motors.get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void powerLeftMotors(double power){
        setWithoutEncoders();
        frontLeft.setPower(power);
        backLeft.setPower(power);
    }

    public void powerRightMotors(double power){
        setWithoutEncoders();
        backRight.setPower(power);
        frontRight.setPower(power);
    }

}
