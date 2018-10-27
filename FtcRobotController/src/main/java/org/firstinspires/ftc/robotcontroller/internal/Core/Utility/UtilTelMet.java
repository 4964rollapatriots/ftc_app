package org.firstinspires.ftc.robotcontroller.internal.Core.Utility;

import org.firstinspires.ftc.robotcore.external.Telemetry;
/**
 * Created by pmkf2 on 10/27/2018.
 */

public final class UtilTelMet
{
    Telemetry telemetry;
    public UtilTelMet(Telemetry TELMET)
    {
        telemetry = TELMET;
    }

    public void addData(String MSG, int VAR)
    {
        telemetry.addData(MSG,VAR);
    }

    public void addData(String MSG, double VAR)
    {
        telemetry.addData(MSG,VAR);
    }

    public void addData(String MSG, boolean VAR)
    {
        telemetry.addData(MSG,VAR);
    }

    public void write(String MSG)
    {
        telemetry.addLine(MSG);
    }

    public void newLine()
    {
        telemetry.addLine();
    }
    public void update()
    {
        telemetry.update();
    }

}
