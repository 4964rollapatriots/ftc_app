package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.I2cDeviceRange;
import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.MRRange;
import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.ODS;
import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.REVColorSensor;
import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.REVIMU;
import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.REVMagnetic;
import org.firstinspires.ftc.robotcontroller.internal.Core.Sensors.Touch;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.CollectorSystem.CollectorSystem;
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
    public CollectorSystem collector = new CollectorSystem();//
    public HookLift latchSystem = new HookLift();
    public PulleyTilt tiltChannel = new PulleyTilt();
    public MarkerDelivery deliver = new MarkerDelivery();
    public Touch bump = new Touch();
    public MRRange frontDistSensor = new MRRange();
    //public MRRange backDistSensor = new MRRange();
    public REVMagnetic hookLimitSwitch = new REVMagnetic();
    public REVMagnetic liftLimitSwitch = new REVMagnetic();

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
        hookLimitSwitch.init(this, "hookMagnet");
        liftLimitSwitch.init(this, "liftMagnet");
        latchSystem.init(this, hookLimitSwitch,liftLimitSwitch);//
        deliver.init(this);
        collector.init(this);//
        tiltChannel.init(this);//
        outTelemetry.write("ABOUT TO RANGE YOU");
        outTelemetry.update();
        //rightDistSensor.mapREVDistanceSensor("rightRange");
        bump.init(this, "touch");
        frontDistSensor.init(this, "frontRange",I2cAddr.create8bit(0x28));
        //backDistSensor.init(this, "backRange", I2cAddr.create8bit(0x10));
        outTelemetry.write("B. I. G. RANGE ");
        outTelemetry.update();
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
        //drivetrain.setDependencies(imu, rightDistSensor,bump, frontDistSensor);
        drivetrain.setDependencies(imu, bump, frontDistSensor);


    }

    public void outTelemetry()
    {
        drivetrain.outTelemetry();
        //tiltChannel.outTelemetry();
        //collector.outTelemetry();
        //latchSystem.outTelemetry();
        //deliver.outTelemetry();
        //outTelemetry.addData("Right Range: ", rightDistSensor.distance());
        //outTelemetry.addData("Front Range: ", frontDistSensor.distance(DistanceUnit.INCH));
        outTelemetry.update();
    }
    @Override
    public void stop()
    {
        drivetrain.stop();
        //collector.stop();
    }
}
