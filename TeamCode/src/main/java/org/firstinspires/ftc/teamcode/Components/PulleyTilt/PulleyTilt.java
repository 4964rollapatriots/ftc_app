package org.firstinspires.ftc.teamcode.Components.PulleyTilt;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.RobotComponent;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by pmkf2 on 11/11/2018.
 */

public class PulleyTilt extends RobotComponent
{

    DcMotor pulleys;

    public double TILT_UP_POW = -1;
    public double TILT_DOWN_POW = 1;

    public int TILT_UP_ENC = 2000; //test this
    public int TILT_DOWN_ENC = 0;

    public int BUFFER = 6;

    public void init(final RobotBase BASE)
    {
        super.init(BASE);
        pulleys =  mapper.mapMotor("pulleys", DcMotorSimple.Direction.FORWARD);
    }

    public void tiltUpByPower()
    {
        pulleys.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleys.setPower(TILT_UP_POW);
    }

    public void tiltDownByPower()
    {
        pulleys.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleys.setPower(TILT_DOWN_POW);
    }

    public void tiltUpByEnc()
    {
        pulleys.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pulleys.setTargetPosition(TILT_UP_ENC);
        pulleys.setPower(TILT_UP_POW);
        if(pulleys.getCurrentPosition() >= TILT_UP_ENC - BUFFER && pulleys.getCurrentPosition() <= TILT_UP_ENC + 7)
        {
            stop();
        }

    }

    public void tiltDownByEnc()
    {
        pulleys.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pulleys.setTargetPosition(TILT_DOWN_ENC);
        pulleys.setPower(TILT_DOWN_POW);
        if(pulleys.getCurrentPosition() >= TILT_DOWN_ENC - BUFFER && pulleys.getCurrentPosition() <= TILT_DOWN_ENC + 7)
        {
            stop();
        }

    }

    public void outTelemetry()
    {
        base.outTelemetry.write("-----------PULLEYS-----------------");
        base.outTelemetry.addData("Pulley Power: ", pulleys.getPower());
        base.outTelemetry.addData("Pulley Position: ", pulleys.getCurrentPosition());
        base.outTelemetry.write("----------END PULLEYS-----------");
        base.outTelemetry.newLine();
    }

    @Override
    public void stop()
    {
        pulleys.setPower(0);
    }
}