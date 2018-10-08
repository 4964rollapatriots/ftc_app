package org.firstinspires.ftc.teamcode.Components.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotComponent;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp.PrototypeRobot;

import java.util.ArrayList;

public class FourMotorDrivetrain extends RobotComponent
{

    private PrototypeRobot bot;
    private HardwareMap map;

    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor frontRight;

    int backLeftEncoder;
    int frontLeftEncoder;
    int backRightEncoder;
    int frontRightEncoder;

    static final double WHEEL_DIAMETER = 100;
    static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    static final double ENCODERS_PER_ROTATION = 150;


    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();

    public FourMotorDrivetrain(PrototypeRobot robot){
        bot = robot;
    }

    public void mapHardware()
    {
        backLeft = map.dcMotor.get("backLeft");
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft = map.dcMotor.get("frontLeft");
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight = map.dcMotor.get("backRight");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight = map.dcMotor.get("frontRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motors.add(backLeft);
        motors.add(backRight);
        motors.add(frontLeft);
        motors.add(frontRight);

    }

    public void setUsingEncoders()
    {
        for (int i = 0; i < motors.size(); i ++)
        {
            motors.get(i).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void resetEncoders()
    {
        for (int i = 0; i < motors.size(); i ++)
        {
            motors.get(i).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void setWithoutEncoders()
    {
        for (int i = 0; i < motors.size(); i ++)
        {
            motors.get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void powerLeftMotors(double power)
    {
        setWithoutEncoders();
        frontLeft.setPower(power);
        backLeft.setPower(power);
    }

    public void powerRightMotors(double power)
    {
        setWithoutEncoders();
        backRight.setPower(power);
        frontRight.setPower(power);
    }

    public void driveToDistance (int cm){

        double rotations =  cm/WHEEL_CIRCUMFERENCE;
        int encoders = (int) (rotations * ENCODERS_PER_ROTATION);

        setUsingEncoders();
        resetEncoders();

        for (int i = 0; i < motors.size(); i ++){
            motors.get(i).setTargetPosition(encoders);
        }
        while (Math.abs(backLeft.getCurrentPosition() - backLeft.getTargetPosition()) < 5){
            powerLeftMotors(0.5);
            powerRightMotors(0.5);
        }

    }

    public void stop()
    {
        for (int i =0;i < motors.size(); i ++)
        {
            motors.get(i).setPower(0);
        }
    }

}
