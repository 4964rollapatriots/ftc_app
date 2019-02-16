package org.firstinspires.ftc.teamcode.Components.CollectorSystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.RobotComponent;

public class CollectorSystem extends RobotComponent {

    public DcMotor collector;
    public DcMotor extendCollector;

    public void init(RobotBase BASE)
    {
        super.init(BASE);
        collector = mapper.mapMotor("collector", DcMotorSimple.Direction.REVERSE);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendCollector = mapper.mapMotor("extend", DcMotorSimple.Direction.REVERSE);
        extendCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    public void powerExtension(double POWER)
    {
        extendCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendCollector.setPower(POWER);
    }

    public void extendLiftByEnc(int encoders)
    {
        extendCollector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendCollector.setTargetPosition(encoders);
        while(!(extendCollector.getCurrentPosition() < encoders + 15 && extendCollector.getCurrentPosition() >= encoders - 15))
        {
            extendCollector.setPower(1);
        }
        extendCollector.setPower(0);
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

    public void releaseBlocks()
    {
        collector.setPower(-1);
    }

    public void releaseBalls()
    {
        collector.setPower(-.65);
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
        extendCollector.setPower(0);
    }
}
