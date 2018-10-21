package org.firstinspires.ftc.teamcode.Components.LanderDump;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.RobotComponent;

/**
 * Created by pmkf2 on 10/20/2018.
 */

public class LanderDump extends RobotComponent
{
    private DcMotor flipOuttake;
    private Servo leftBucket;
    private Servo rightBucket;

    private final double BUCKET_DUMP_POS = .60;
    private final double RESET_BUCKET_POS = 0;
    private final double FLIP_POWER = .50;

    private final int FLIP_POS = 2000; //guestimate
    private final int STANDARD_POS = 0; //guestimate


    public void init(RobotBase BASE)
    {
        super.init(BASE);
        flipOuttake = mapper.mapMotor("flipOuttake", DcMotorSimple.Direction.FORWARD); //check this
        flipOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBucket = mapper.mapServo("leftBucket", Servo.Direction.FORWARD);
        rightBucket = mapper.mapServo("rightBucket", Servo.Direction.FORWARD);
    }

    public void flip()
    {
        flipOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipOuttake.setTargetPosition(FLIP_POS);
        flipOuttake.setPower(FLIP_POWER);
    }

    public void unFlip()
    {
        flipOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipOuttake.setTargetPosition(STANDARD_POS);
        flipOuttake.setPower(.60);
    }

    public void dumpBothBuckets()
    {
        rightBucket.setPosition(BUCKET_DUMP_POS);
        leftBucket.setPosition(BUCKET_DUMP_POS);
    }

    public void dumpLeftBucket()
    {
        leftBucket.setPosition(BUCKET_DUMP_POS);
    }

    public void dumpRightBucket()
    {
        rightBucket.setPosition(BUCKET_DUMP_POS);
    }

    public void resetBothBuckets()
    {
        leftBucket.setPosition(RESET_BUCKET_POS);
        rightBucket.setPosition(RESET_BUCKET_POS);
    }

    @Override
    public void stop()
    {
        flipOuttake.setPower(0);
    }
}
