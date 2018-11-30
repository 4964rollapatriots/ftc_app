package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.REVIMU;
import org.firstinspires.ftc.teamcode.Components.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Components.HookLift.HookLift;
import org.firstinspires.ftc.teamcode.Components.MarkerDelivery.MarkerDelivery;
import org.firstinspires.ftc.teamcode.Components.PulleyTilt.PulleyTilt;

import static java.lang.Thread.sleep;

/*
    This class is essentially the robot manager.
 */
public class Base extends RobotBase
{
    public Drivetrain drivetrain = new Drivetrain();
    public REVIMU imu = new REVIMU();
    //public CollectorSystem collector = new CollectorSystem();//
    public HookLift latchSystem = new HookLift();
    public PulleyTilt tiltChannel = new PulleyTilt();
    public MarkerDelivery deliver = new MarkerDelivery();

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
        imu.init(this, "imu", params);
        drivetrain.init(this , imu);
        deliver.init(this);
        latchSystem.init(this);//
        //collector.init(this);//
        tiltChannel.init(this);//
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
        //tiltChannel.outTelemetry();
        //collector.outTelemetry();
        //latchSystem.outTelemetry();
        deliver.outTelemetry();
        outTelemetry.update();
    }
    @Override
    public void stop()
    {
        drivetrain.stop();
        //collector.stop();
    }
}
