package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;

public class Robot {

    private FourMotorDrivetrain drivetrain;

    private String name;
    private final String DEFAULT_NAME = "My Robot";

    public Robot(String s){
        name = s;
    }

    public Robot(){
        name = DEFAULT_NAME;
    }

    public String name(){
        return name;
    }

    public void init(){
        drivetrain = new FourMotorDrivetrain(this);
        drivetrain.mapHardware();
        drivetrain.stop();
    }
}
