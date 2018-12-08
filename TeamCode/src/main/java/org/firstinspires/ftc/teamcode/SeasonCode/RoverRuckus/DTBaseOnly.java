package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus;

/**
 * Created by Pranal Madria on 11/27/2018.
 */
//We will use this base to only test the drivetrain related classes. This will be convenient
    //as we won't need to always comment the rest of the robot out in Base.java when we aren't using it.

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.REVIMU;
import org.firstinspires.ftc.teamcode.Components.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Components.MarkerDelivery.MarkerDelivery;

import static java.lang.Thread.sleep;



//We will use this base to only test the drive-train related classes. This will be convenient
//as we won't need to always comment the rest of the robot out in Base.java when we aren't using it.
public class DTBaseOnly extends RobotBase
{
    public Drivetrain drivetrain = new Drivetrain();
    public REVIMU imu = new REVIMU();
    public MarkerDelivery deliver = new MarkerDelivery();

    @Override
    public void init(final HardwareMap hwm, final LinearOpMode op)
    {
        super.init(hwm, op);
        drivetrain.init(this,imu);
        deliver.init(this);
        //Try catches to prevent crashes
        //imu.write8(BNO055IMU.Register.OPR_MODE ,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        try
        {
            sleep(100);
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }



        // Dependency setting
        drivetrain.setDependencies(imu);

    }

    public void outTelemetry()
    {
        drivetrain.outTelemetry();
        deliver.outTelemetry();
        outTelemetry.update();
    }
    @Override
    public void stop()
    {
        drivetrain.stop();
    }
}

