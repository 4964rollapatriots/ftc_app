package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Components.Drivetrain.FourMotorDrivetrain;


public class PrototypeRobot {


    public HardwareMap hardware;
    public FourMotorDrivetrain drivetrain;
    //public Collector collector;

    private String name;
    private final String DEFAULT_NAME = "My Robot";

    public PrototypeRobot(String name, HardwareMap hw){
        this.name = name;
        hardware = hw;
    }

    public PrototypeRobot(){
        name = DEFAULT_NAME;
    }

    public String name(){
        return name;
    }

    public void init(){
        drivetrain = new FourMotorDrivetrain(this);
        drivetrain.mapHardware();
        drivetrain.stop();


//        collector = new Collector(this);
//        collector.mapHardware();
//        collector.stop();

    }
}
