package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Drivetrain.FourMotorDrivetrain;

public class PrototypeRobot {

    public FourMotorDrivetrain drivetrain;
    //public Collector collector;

    private String name;
    private final String DEFAULT_NAME = "My Robot";

    public PrototypeRobot(String name){
        this.name = name;
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
