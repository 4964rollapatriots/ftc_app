package org.firstinspires.ftc.robotcontroller.internal.Core;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class RobotCommand
{

    public Telemetry telemetry;

    protected RobotComponent component = null;

    protected Thread t = null;

    public RobotCommand()
    {

    }

    //Constructor - ties the command to a component.

    public RobotCommand(RobotComponent COMPONENT)
    {
        component = COMPONENT;
    }

    //Runs the command on the main thread.
    public abstract void runSequentially();

    //Runs command parallel to main thread.
    public abstract void runParallel();

    //Stops current command execution
    public abstract void stop();
}
