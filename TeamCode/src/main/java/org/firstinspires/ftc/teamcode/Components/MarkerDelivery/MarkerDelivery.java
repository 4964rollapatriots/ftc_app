package org.firstinspires.ftc.teamcode.Components.MarkerDelivery;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.RobotComponent;

/**
 * Created by pmkf2 on 11/4/2018.
 */

public class MarkerDelivery extends RobotComponent
{
    public CRServo markerDelivery;


    public final double RAISE = -1;
    public final double DELIVERY = 1;

    public void init(final RobotBase BASE)
    {
        super.init(BASE);
        markerDelivery = mapper.mapCRServo("marker", CRServo.Direction.FORWARD);
    }

    public void deliverMarker()
    {
        markerDelivery.setPower(DELIVERY);
    }

    public void raiseMarker()
    {
        markerDelivery.setPower(RAISE);
    }


    public void outTelemetry()
    {
        base.outTelemetry.write("----------------MARKER DELIVERY---------------");
        base.outTelemetry.addData("Delivery Power", markerDelivery.getPower());
        base.outTelemetry.write("-------------END MARKER DELIVERY--------------");
        base.outTelemetry.newLine();
    }
    @Override
    public void stop()
    {
        markerDelivery.setPower(0);
    }
}
