package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.REVIMU;
import org.firstinspires.ftc.teamcode.Components.Collector.Collector;
import org.firstinspires.ftc.teamcode.Components.Drivetrain.Drivetrain;

import static java.lang.Thread.sleep;

/*
    This class is essentially the robot manager.
 */
public class Base extends RobotBase
{
    public Drivetrain drivetrain = new Drivetrain();

    public REVIMU imu = new REVIMU();

    public Collector collector = new Collector();

    @Override
    public void init(final HardwareMap hwm, final LinearOpMode op)
    {
        super.init(hwm, op);
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();

        // IMU parameters
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled = true;
        params.loggingTag = "IMU";


        // Basic component initialization
        drivetrain.init(this);
        imu.init(this, "imu", params);
        collector.init(this);

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
    @Override
    public void stop()
    {
        drivetrain.stop();
        collector.stop();
    }
}
