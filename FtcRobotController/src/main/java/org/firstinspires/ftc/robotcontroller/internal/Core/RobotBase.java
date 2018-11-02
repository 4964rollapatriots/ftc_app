package org.firstinspires.ftc.robotcontroller.internal.Core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.UtilTelMet;

/**
 *
 * Team 4964 Rolla Patriots
 *
 * Authors: Joel Schott and now Pranal Madria ;)
 * Created on 8/8/2018.
 */

public abstract class RobotBase
{



    public HardwareMap hardwareMap;

    public LinearOpMode opMode;

    public UtilTelMet outTelemetry;

    public final LinearOpMode opmode(){
        return opMode;
    }



    public void init (final HardwareMap hwm, final LinearOpMode op)
    {
        hardwareMap = hwm;
        outTelemetry = new UtilTelMet(op.telemetry);
        opMode = op;
    }

    public abstract void stop();

}
