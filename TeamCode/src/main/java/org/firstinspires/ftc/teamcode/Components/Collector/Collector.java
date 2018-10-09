package org.firstinspires.ftc.teamcode.Components.Collector;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.RobotComponent;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp.PrototypeRobot;

public class Collector extends RobotComponent {



    private DcMotor collector;
    private DcMotor collectorPivot;

    public void init(RobotBase BASE)
    {
        super.init(BASE);
        collector = mapper.mapMotor("collector", DcMotorSimple.Direction.FORWARD);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collectorPivot = mapper.mapMotor("pivot", DcMotorSimple.Direction.REVERSE);
        collectorPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    //Run the actual collector motor
    public void runCollector(double SPEED)
    {
        collector.setPower(SPEED);
    }

    //Move actual collector pivot. If true is passed in, move collector down. If false
    //is passed in, move collector up
    public void putCollectorDown(boolean DOWN)
    {
        if(DOWN)
        {
            collectorPivot.setPower(.20);
        }
    }

    public void putCollectorUp(boolean UP)
    {
        if(UP)
        {
            collectorPivot.setPower(-.20);
        }
    }

    @Override
    public void stop()

    {
        collector.setPower(0);
        collectorPivot.setPower(0);
    }
}
