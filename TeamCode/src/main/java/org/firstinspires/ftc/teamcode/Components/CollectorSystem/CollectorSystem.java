package org.firstinspires.ftc.teamcode.Components.CollectorSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.RobotComponent;

public class CollectorSystem extends RobotComponent {

    private Servo leftSlide;
    private Servo rightSlide;
    private DcMotor collector;

    public void init(RobotBase BASE)
    {
        super.init(BASE);
        collector = mapper.mapMotor("collector", DcMotorSimple.Direction.FORWARD);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide = mapper.mapServo("left_slide", Servo.Direction.FORWARD);
        rightSlide = mapper.mapServo("right_slide", Servo.Direction.FORWARD);
    }


    public void extendLifts()
    {
        leftSlide.setPosition(leftSlide.getPosition() + .15);
        rightSlide.setPosition(rightSlide.getPosition() + .15);
    }

    public void retractLifts()
    {
        leftSlide.setPosition(leftSlide.getPosition() - .15);
        rightSlide.setPosition(rightSlide.getPosition() - .15);
    }

    //Run the actual collector motor
    public void runCollector(double SPEED)
    {
        collector.setPower(SPEED);
    }

    public void outTelemetry()
    {
        base.outTelemetry.write("----------COLLECTOR SYSTEM-----------");
        base.outTelemetry.addData("Collector Power", collector.getPower());
        base.outTelemetry.addData("Left Slide Position", leftSlide.getPosition());
        base.outTelemetry.addData("Right Slide Position", rightSlide.getPosition());
        base.outTelemetry.write("----------END COLLECTOR SYSTEM---------");
        base.outTelemetry.newLine();
    }
    @Override
    public void stop()

    {
        collector.setPower(0);
    }
}
