package org.firstinspires.ftc.robotcontroller.internal.Core.Utility;

public final class Util
{
    public static int angleError(final int CURRENT, double target)
    {
        //Hold Target Value
        double initTarget = target;

        int error = 0;

        //Set the angle to be between 0/360
        if(initTarget < CURRENT)
            target += 360;

        //Find Error
        error = (int) target - CURRENT;


        //Really weird edge case need to contain
        if((CURRENT == 2 || CURRENT == 1 || CURRENT == 0) && error < -300)
        {
                error += 360;
                return error;
        }
        //Do some math with error to find correct sign(positive or negative)
        if(Math.abs(error) > 180)
        {
            error = Math.abs(360 - error);
        }

        //Another function to put the angle in correct form due to orientation
        if(standardizeAngle(error + CURRENT) != initTarget)
        {
            error *= -1;
        }

        return error;
    }

    public static int standardizeAngle(int angle)
    {

        angle %= 360;
        if(angle < 0)
        {
            angle += 360;
        }
        return angle;
    }

    public static double scaleValue(final double VALUE)
    {
        double result = 0;

        result = Math.pow(VALUE, 2);

        if(result != 0)
        {
            result *= VALUE / Math.abs(VALUE);
        }

        return result;
    }
}
