package org.firstinspires.ftc.teamcode.Components.LanderLift;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.RobotComponent;

/**
 * Created by pmkf2 on 10/20/2018.
 */

public class LanderLift extends RobotComponent
{

    /*
    MOST LIKELY WON'T BE USING THIS CLASS AS WE AREN'T USING THIS DESIGN ON OUR ROBOT ANYMORE
     */
    private DcMotor RevLIFTS;
    private Servo bucketDoor;

    private final double OPEN = .75;
    private final double CLOSED = 0;

    private final double EXTEND = .75;
    private final double RETRACT = -.75;

    private final int EXTEND_BY_POS = 2000; //extend by position guestimate
    private final int RETRACT_BY_POS = -2000; //extend by retraction guestimate

    public void init(RobotBase BASE)
    {
        super.init(BASE);
        RevLIFTS = mapper.mapMotor("RevLIFTS", DcMotorSimple.Direction.FORWARD); //check this
        RevLIFTS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bucketDoor = mapper.mapServo("bucketDoor", Servo.Direction.FORWARD);
    }

    public void extendByPower()
    {
        RevLIFTS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RevLIFTS.setPower(EXTEND);
    }

    public void retractByPower()
    {
        RevLIFTS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RevLIFTS.setPower(RETRACT);
    }

    public void extendByEncoder()
    {
        RevLIFTS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RevLIFTS.setTargetPosition(EXTEND_BY_POS);
        RevLIFTS.setPower(EXTEND);
    }

    public void retractByEncoder()
    {
        RevLIFTS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RevLIFTS.setTargetPosition(RETRACT_BY_POS);
        RevLIFTS.setPower(RETRACT);
    }

    public void openBucketDoor()
    {
        bucketDoor.setPosition(OPEN);
    }

    public void closeBucket()
    {
        bucketDoor.setPosition(CLOSED);
    }

    public void outTelemetry()
    {
        base.outTelemetry.addData("-------------------LANDER LIFT---------------", true);
    }

    @Override
    public void stop()
    {
        RevLIFTS.setPower(0);
    }
}
