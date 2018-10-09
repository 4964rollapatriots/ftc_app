package org.firstinspires.ftc.teamcode.Components.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.RobotComponent;
import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.REVIMU;

public class Drivetrain extends RobotComponent
{
    //CONSTANTS

    private final double FORWARD = 1;
    private final double BACKWARD = -1;
    private final double SLOW = .75;
    private final double FAST = .95;
    private final double STOP = 0;

    private double _powerMult = STOP;

    //This will store each of the motors' encoder counts in a static array. No need for a
    //dynamic one. The order is backLeft, backRight, frontLeft, frontRight.
    private long[] encoderCounts = {0, 0, 0, 0};

    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    private REVIMU imu;

    //Instantiate Commands



    private State currState = State.STOP;

    public enum State
    {
        FORWARD_FAST,
        FORWARD_SLOW,
        INVERTED_FAST,
        INVERTED_SLOW,
        STOP
    }

    public void init(final RobotBase BASE)
    {
        super.init(BASE);

        backLeft = mapper.mapMotor("backLeft", DcMotorSimple.Direction.FORWARD);
        frontLeft = mapper.mapMotor("frontLeft", DcMotorSimple.Direction.FORWARD);
        backRight = mapper.mapMotor("backRight", DcMotorSimple.Direction.REVERSE);
        frontRight = mapper.mapMotor("frontRight", DcMotorSimple.Direction.REVERSE);

    }

    public void setDependencies(final REVIMU IMU)
    {
        imu = IMU;
    }
    public long backLeftEncoderCount()
    {
        return backLeft.getCurrentPosition();
    }

    public long backRightEncoderCount()
    {
        return backRight.getCurrentPosition();
    }

    public long frontLeftEncoderCount()
    {
        return frontLeft.getCurrentPosition();
    }

    public long frontRightEncoderCount()
    {
        return frontRight.getCurrentPosition();
    }

    public long[] getEncoderCounts()
    {
        encoderCounts[0] = backLeftEncoderCount();
        encoderCounts[1] = backRightEncoderCount();
        encoderCounts[2] = frontLeftEncoderCount();
        encoderCounts[3] = frontRightEncoderCount();
        return encoderCounts;
    }

    public void encoderToPos()
    {
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void run(double drivePower, double rotatePower, boolean inverted, boolean scale)
    {
        drivePower *= _powerMult;
        rotatePower *= _powerMult;

        if(currState == State.INVERTED_FAST || currState == State.INVERTED_SLOW)
        {
            rotatePower *= _powerMult;
        }
        backRight.setPower(drivePower + rotatePower);
        backLeft.setPower(drivePower - rotatePower);
        frontRight.setPower(drivePower + rotatePower);
        frontLeft.setPower(drivePower - rotatePower);
    }

    public void encoderOn()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void encoderOff()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void encoderStopReset()
    {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public State state()
    {
        return currState;
    }

    public void setCurrState(final State STATE)
    {
        switch(STATE)
        {
            case FORWARD_FAST:
                _powerMult = FORWARD * FAST;
                encoderOff();
                break;
            case FORWARD_SLOW:
                _powerMult = FORWARD * SLOW;
                encoderOff();
                break;
            case INVERTED_FAST:
                _powerMult = BACKWARD * FAST;
                encoderOff();
                break;
            case INVERTED_SLOW:
                _powerMult = BACKWARD * SLOW;
                encoderOff();
                break;
            case STOP:
                _powerMult = STOP;
                encoderStopReset();
                break;
        }
    }

    public DcMotor.RunMode getEncoderMode()
    {
        //assumes that all other motors are in the same mode. It is crucial to set all the motors to the same mode each
        //time a mode changes.
        return backLeft.getMode();
    }

    public DcMotor backLeft()
    {
        return backLeft;
    }

    public DcMotor backRight()
    {
        return backRight;
    }


    public DcMotor frontLeft()
    {
        return frontLeft;
    }

    public DcMotor frontRight()
    {
        return frontRight;
    }

    public void setAllMotorPower(double SPEED)
    {
        backLeft.setPower(SPEED);
        backRight.setPower(SPEED);
        frontLeft.setPower(SPEED);
        frontRight.setPower(SPEED);
    }

    @Override

    public void stop()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

}
