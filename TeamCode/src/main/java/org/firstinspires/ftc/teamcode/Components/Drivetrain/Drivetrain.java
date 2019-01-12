package org.firstinspires.ftc.teamcode.Components.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.RobotComponent;
import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.MRRange;
import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.REVIMU;
import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.Touch;
import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.Util;

public class Drivetrain extends RobotComponent
{
    //CONSTANTS

    private final double FORWARD = 1;
    private final double BACKWARD = -1;
    private final double SLOW = .50;
    private final double FAST = 1;
    private final double STOP = 0;

    private double _powerMult = STOP;


    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    public REVIMU imu;
    public MRRange range;
    public Touch touch;

    //Instantiate Commands
    public TurnTo turnTo;
    public DriveToDistance driveTo ;

    private State currState = State.STOP;

    public enum State
    {
        FORWARD_FAST,
        FORWARD_SLOW,
        INVERTED_FAST,
        INVERTED_SLOW,
        STOP
    }

    public void init(final RobotBase BASE, REVIMU baseIMU)
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
        turnTo = new TurnTo(this, imu);
        driveTo = new DriveToDistance(this);
    }

    public void setDependencies(final REVIMU IMU, final MRRange RANGE){

        imu = IMU;
        range = RANGE;
        turnTo = new TurnTo(this, imu);
        driveTo = new DriveToDistance(this);
    }

    public void setDependencies(final REVIMU IMU, final MRRange RANGE, final Touch t){

        imu = IMU;
        range = RANGE;
        touch = t;
        turnTo = new TurnTo(this, imu);
        driveTo = new DriveToDistance(this);
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
        //This will store each of the motors' encoder counts in a static array. No need for a
        //dynamic one. The order is backLeft, backRight, frontLeft, frontRight.
        long[] encoderCounts = {0, 0, 0, 0};
        encoderCounts[0] = backLeftEncoderCount();
        encoderCounts[1] = backRightEncoderCount();
        encoderCounts[2] = frontLeftEncoderCount();
        encoderCounts[3] = frontRightEncoderCount();
        return encoderCounts;
    }
    public long getAverageEncoderCounts()
    {
        long[] encoderArr = getEncoderCounts();
        int total = 0;
        for(int i = 0; i < encoderArr.length; i++)
        {
            total += encoderArr[i];
        }
        return (total/encoderArr.length);
    }
    public long[] getTargetEncoderCounts()
    {
        long[] targetCounts = {0,0,0,0};
        targetCounts[0] = backLeft.getTargetPosition();
        targetCounts[1] = backRight.getTargetPosition();
        targetCounts[2] = frontLeft.getTargetPosition();
        targetCounts[3] = frontRight.getTargetPosition();
        return targetCounts;
    }

    public void setEncoderState(DcMotor.RunMode state)
    {
        backLeft.setMode(state);
        backRight.setMode(state);
        frontLeft.setMode(state);
        frontRight.setMode(state);
    }


    public void encoderToPos()
    {
        setEncoderState(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void arcRun(double leftPower,double rightPower)
    {
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        backRight.setPower(rightPower);
        frontRight.setPower(rightPower);
    }
    public void run(double drivePower, double rotatePower, boolean scale)
    {

        if(scale)
        {
            drivePower = Util.scaleValue(drivePower);
            rotatePower = Util.scaleValue(rotatePower);
        }
        drivePower *= _powerMult;
        rotatePower *= -1 * _powerMult;
        if(currState == State.INVERTED_FAST || currState == State.INVERTED_SLOW) {
            rotatePower *= -1;
        }
        backRight.setPower(drivePower + rotatePower);
        backLeft.setPower(drivePower - rotatePower);
        frontRight.setPower(drivePower + rotatePower);
        frontLeft.setPower(drivePower - rotatePower);
    }

    public void runTeleOp(double drivePower, double rotatePower)
    {

//        drivePower *= _powerMult;
//        rotatePower *= -1 * _powerMult;
//        if(inverted)
//        {
//            setCurrState(State.INVERTED_FAST);
//            if (slowMode)
//            {
//                setCurrState(State.INVERTED_SLOW);
//            }
//        }
//        else
//        {
//            setCurrState(State.FORWARD_FAST);
//            if(slowMode)
//            {
//                setCurrState(State.FORWARD_SLOW);
//            }
//        }
//        if(currState == State.INVERTED_FAST || currState == State.INVERTED_SLOW)
//        {
//            rotatePower *= -1;
//        }

        backRight.setPower(drivePower - rotatePower);
        backLeft.setPower(drivePower + rotatePower);
        frontRight.setPower(drivePower - rotatePower);
        frontLeft.setPower(drivePower + rotatePower);
    }
    public void encoderOn()
    {
        setEncoderState(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderOff()
    {
        setEncoderState(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void encoderStopReset()
    {
        setEncoderState(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        currState = STATE;
    }


    public DcMotor.RunMode getEncoderMode()
    {
        //assumes that all other motors are in the same mode. It is crucial to set all the motors to the same mode each
        //time a mode changes.
        return backLeft.getMode();
    }

    public Boolean isBusy()
    {
        return (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy());
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

    public void testDriveForEncoders(int encoders){
        encoderOn();
        encoderStopReset();
        encoderToPos();
        backLeft.setTargetPosition(encoders);
        backRight.setTargetPosition(encoders);
        frontLeft.setTargetPosition(encoders);
        frontRight.setTargetPosition(encoders);
        frontLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(0.5);
        while (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy() )
        {

        }
        this.stop();
    }

    public void outTelemetry()
    {
        base.outTelemetry.write("-----------DRIVETRAIN-----------");
        base.outTelemetry.addData("Back Left Power: ", backLeft.getPower());
        base.outTelemetry.addData("Back Right Power: ", backRight.getPower());
        base.outTelemetry.addData("Front Left Power: ", frontLeft.getPower());
        base.outTelemetry.addData("Front Right Power: ", frontRight.getPower());
        base.outTelemetry.newLine();
        base.outTelemetry.addData("Back Left Encoder: ", backLeftEncoderCount());
        base.outTelemetry.addData("Front Left Encoder: ", frontLeftEncoderCount());
        base.outTelemetry.addData("Back Right Encoder: ", backRightEncoderCount());
        base.outTelemetry.addData("Front Right Encoder: ", frontRightEncoderCount());
        base.outTelemetry.addData("IMU Z Angle: ", imu.zAngle());
        base.outTelemetry.write("-----END DRIVETRAIN -------------");
        base.outTelemetry.newLine();
    }
    public double testTurnTo(double targetAngle){
        encoderOff();
        double leftPower = 0;
        double rightPower = 0;
        imu.calibrateTo(0);
        imu.setAngle();
        double heading = imu.zAngle();
        while (Math.abs(heading - targetAngle)<5){
            imu.setAngle();
            heading = imu.zAngle();
            if ((heading - targetAngle) < 180){
                leftPower = 0.5;
                rightPower = -0.5;
            }
            else{
                leftPower = -0.5;
                rightPower = 0.5;
            }
            backLeft.setPower(leftPower);
            frontLeft.setPower(leftPower);
            backRight.setPower(rightPower);
            frontRight.setPower(rightPower);
        }
        this.stop();
        return heading;
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
