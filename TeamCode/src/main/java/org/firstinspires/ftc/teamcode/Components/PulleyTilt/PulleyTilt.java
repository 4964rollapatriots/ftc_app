
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

    public DcMotor pulleys;

    private double TILT_UP_POW = -.80;
    private double TILT_DOWN_POW = .80;

    private int TILT_UP_ENC = -550; //test this
    private int TILT_DOWN_ENC = 1550;

    public int BUFFER = 75;

    public void init(final RobotBase BASE)
    {
        super.init(BASE);
        pulleys =  mapper.mapMotor("pulleys", DcMotorSimple.Direction.FORWARD);
    }

    public void tiltByPower(double POWER)
    {
        pulleys.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleys.setPower(POWER);
    }

    public boolean tiltUpByEnc()
    {
        //Run without encoder, use custom motion profile
        pulleys.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Pulley position for testing
        base.outTelemetry.addData("Position: ", pulleys.getCurrentPosition());
        base.outTelemetry.update();

        //If already there, stop
        if (pulleys.getCurrentPosition() <= TILT_UP_ENC )
        {
            stop();
            return true;
        }
        //Dramitically slow down
        else if(Math.abs(pulleys.getCurrentPosition() - TILT_UP_ENC) < 300)
        {
            //TILT_UP_POW *= .95;
            pulleys.setPower(TILT_UP_POW * .40);
        }
        //Slow Down a bit
        else if(Math.abs(pulleys.getCurrentPosition() - TILT_UP_ENC) < 500)
        {
            pulleys.setPower(TILT_UP_POW * .70);
        }
        //Set Power to Full
        else
        {
            pulleys.setPower(TILT_UP_POW);
        }
        return false;
    }

    public boolean tiltDownByEnc()
    {
        pulleys.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        base.outTelemetry.addData("Position: ", pulleys.getCurrentPosition());
        base.outTelemetry.update();
        if (pulleys.getCurrentPosition() >= TILT_DOWN_ENC )
        {
            base.outTelemetry.addData("Got Inside the IF Statemnt for TILT UP", true);
            base.outTelemetry.update();
            stop();
            return true;
        }
        else if(Math.abs(pulleys.getCurrentPosition() - TILT_DOWN_ENC) < 900)
        {
            //TILT_UP_POW *= .95;
            pulleys.setPower(TILT_DOWN_POW * .70);
        }
        else if(Math.abs(pulleys.getCurrentPosition() - TILT_DOWN_ENC) < 300)
        {
            pulleys.setPower(TILT_DOWN_POW * .40);
        }
        else {
            pulleys.setPower(TILT_DOWN_POW);
        }
        return false;

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
