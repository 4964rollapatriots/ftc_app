package org.firstinspires.ftc.teamcode.Components.MarkerDelivery;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.RobotComponent;

/**
 * Created by pmkf2 on 11/4/2018.
 */

public class MarkerDelivery extends RobotComponent
{
    public Servo markerDelivery;


    public final double RAISE = .75;
    public final double DELIVERY = 0;

    public void init(final RobotBase BASE)
    {
        super.init(BASE);
        markerDelivery = mapper.mapServo("marker", Servo.Direction.FORWARD);
    }

    public void deliverMarker()
    {
        markerDelivery.setPosition(DELIVERY);
    }

    public void raiseMarker()
    {
        markerDelivery.setPosition(RAISE);
    }

    public void outTelemetry()
    {
        base.outTelemetry.write("----------------MARKER DELIVERY---------------");
        base.outTelemetry.addData("Delivery Position", markerDelivery.getPosition());
        base.outTelemetry.write("-------------END MARKER DELIVERY--------------");
        base.outTelemetry.newLine();
    }
    @Override
    public void stop()
    {
        raiseMarker();
    }
}
