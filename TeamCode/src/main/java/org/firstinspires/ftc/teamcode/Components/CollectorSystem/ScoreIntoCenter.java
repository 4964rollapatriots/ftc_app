package org.firstinspires.ftc.teamcode.Components.CollectorSystem;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotCommand;
import org.firstinspires.ftc.teamcode.Components.PulleyTilt.PulleyTilt;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;

/**
 * Created by Pranal on 12/4/2018.
 */

public class ScoreIntoCenter extends RobotCommand
{

    private CollectorSystem _collector;
    private PulleyTilt _tilt;
    private double speed = 1;
    private final int ENC_COUNTS = 2000;

    private boolean _busy = false;
    private boolean endCommand = false;

    public enum TiltState{UP, DOWN}

    public ScoreIntoCenter()
    {

    }

    public ScoreIntoCenter (CollectorSystem COLLECTOR)
    {
        _collector = COLLECTOR;
    }

    public void init(CollectorSystem COLLECTOR)
    {
        _collector = COLLECTOR;
    }

    public void rotateUp()
    {
        _tilt.tiltUpBallByEnc();
    }

    public void rotateDown()
    {
        _tilt.teleOpTiltDownByEnc();
    }
    @Override
    public void runParallel()
    {

    }

    @Override
    public void runSequentially()
    {

    }


    @Override
    public void stop()
    {

    }
}
