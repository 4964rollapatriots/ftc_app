package org.firstinspires.ftc.teamcode.Components.CollectorSystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.RobotComponent;

public class CollectorSystem extends RobotComponent {

    private DcMotor collector;
    private DcMotor extendCollector;
    private CRServo particleRelease;

    public void init(RobotBase BASE)
    {
        super.init(BASE);
        collector = mapper.mapMotor("collector", DcMotorSimple.Direction.REVERSE);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendCollector = mapper.mapMotor("extend", DcMotorSimple.Direction.REVERSE);
        extendCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        particleRelease = mapper.mapCRServo("release", CRServo.Direction.FORWARD);
    }


    public void powerLift(double POWER)
    {
        extendCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendCollector.setPower(POWER);
    }

    public void extendLiftByEnc()
    {
        extendCollector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void retractLiftByEnc()
    {
        extendCollector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Run the actual collector motor
    public void runCollector(double SPEED)
    {
        collector.setPower(SPEED);
    }

    public void releaseParticles()
    {
        particleRelease.setPower(.4);
    }

    public void stopCRServo()
    {
        particleRelease.setPower(0);
    }
    public void containParticles()
    {
        particleRelease.setPower(-.4);
    }
    public void outTelemetry()
    {
        base.outTelemetry.write("----------COLLECTOR SYSTEM-----------");
        base.outTelemetry.addData("Collector Power: ", collector.getPower());
        base.outTelemetry.addData("Collector Extension Power: ", extendCollector.getPower());
        base.outTelemetry.write("----------END COLLECTOR SYSTEM---------");
        base.outTelemetry.newLine();
    }
    @Override
    public void stop()

    {
        collector.setPower(0);
    }
}
