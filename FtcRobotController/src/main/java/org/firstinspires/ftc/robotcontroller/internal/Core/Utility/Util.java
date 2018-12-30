package org.firstinspires.ftc.robotcontroller.internal.Core.Utility;

public final class Util
{
    public static int angleError(final int CURRENT, double target)
    {
        double initTarget = target;

        int error = 0;

        if(initTarget < CURRENT)
            target += 360;

        error = (int) target - CURRENT;


        if(CURRENT == 1 && error < -300)
        {
                error += 360;
                return error;
        }
        if(Math.abs(error) > 180)
        {
            error = Math.abs(360 - error);
        }

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
