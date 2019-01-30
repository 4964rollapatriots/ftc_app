    package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Components.Drivetrain.TurnTo;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;

    @TeleOp(name = "MainTeleOp", group = "TeleOp")
public class TeleOpMain extends LinearOpMode
{
    //create an instance of our base which contains all of the components of the robot
    private Base _base = new Base();
    //Constants for manipulating the collector's lift power
    private double EXTEND_LIFT_POW = 1;
    private double RETRACT_LIFT_POW = -1;
    private boolean tilt = false;
    private boolean scaleMode = false;
    private boolean slowMode = false;
    private boolean inverted = false;
    private boolean invertedControl = false;
    private boolean scaleModeControl = false;
    private boolean slowModeControl = false;
    private boolean tiltControl = false;
    private boolean hookOpen = true;
    private boolean up = false;
    private boolean down = false;
    private boolean automateLift = false;
    private boolean finalLift = false;
    private boolean releaseRightTrigger = true;
    private double drivetrainScale = 1;
    private boolean autoTurntoLander = false;
    private boolean autoTurntoCrater = false;
    private boolean imuCalibrated = false;

    private boolean bHeld = false;
    private boolean aHeld = false;

    public void runOpMode() throws InterruptedException
    {
        _base.init(hardwareMap, this);

        _base.deliver.markerDelivery.setPosition(0);
        waitForStart();

        //Set State of the Drivetrain
        _base.drivetrain.setCurrState(Drivetrain.State.FORWARD_FAST);

        //run teleop while opmode is active
        while(opModeIsActive())
        {
            run();
        }
    }
    //The Actual Teleop commands
    private void run() {
        /*----------------------------------- STANDARD DRIVING ----------------------------------*/
        //drive the robot using gamepad1 joysticks, standard six wheel movement

//        if (gamepad1.right_bumper) {
//            if (invertedControl) {
//                invertedControl = false;
//                inverted = !inverted;
//            }
//        } else
//        {
//            invertedControl = true;
//        }

        if(gamepad1.left_bumper) {
            if (slowModeControl) {
                slowModeControl = false;
                slowMode = !slowMode;
            }
        }
        else
            slowModeControl = true;

        if(gamepad1.a) {
            if (scaleModeControl) {
                scaleModeControl = false;
                scaleMode = !scaleMode ;
            }
        }
        else
            scaleModeControl = true;

        if (gamepad2.right_bumper) {
            if (tiltControl) {
                tiltControl= false;
                tilt = !tilt;
            }
        } else
        {
            tiltControl= true;
        }

        if(inverted)
        {
            if(slowMode)
            {
                _base.drivetrain.setCurrState(Drivetrain.State.INVERTED_SLOW);
            }
            else
            {
                _base.drivetrain.setCurrState(Drivetrain.State.INVERTED_FAST);
            }
        }
        else
        {
            if(slowMode)
            {
                _base.drivetrain.setCurrState(Drivetrain.State.FORWARD_SLOW);
            }
            else
            {
                _base.drivetrain.setCurrState(Drivetrain.State.FORWARD_FAST);
            }
        }
        if (gamepad1.right_bumper && releaseRightTrigger){
            finalLift = !finalLift;
            releaseRightTrigger = false;
        }
        if (! gamepad1.right_bumper){
            releaseRightTrigger = true;
        }
        if (finalLift){
            drivetrainScale = 0.3;
        }
        else{
            drivetrainScale = 1;
        }

        _base.drivetrain.run(-com.qualcomm.robotcore.util.Range.clip(gamepad1.left_stick_y, -1, 1),
                com.qualcomm.robotcore.util.Range.clip(gamepad1.right_stick_x, -1, 1), scaleMode,drivetrainScale);



        if (( gamepad1.right_stick_button) && !bHeld){
            if (!imuCalibrated){
                _base.imu.calibrateTo(0);
                imuCalibrated = true;
            }
            bHeld = true;
        }
        if ( !gamepad1.right_stick_button){
            bHeld = false;
        }
        if (gamepad1.left_stick_y > 0.1 && gamepad1.right_stick_x > 0.1){
            autoTurntoLander = false;
        }
        if (autoTurntoLander){

            _base.drivetrain.turnTo.goTo(180,0.9);
            if (_base.drivetrain.turnTo.teleopRunSequentially(4, 5)){
                autoTurntoLander = false;
                _base.drivetrain.stop();
            }

            if (gamepad1.left_stick_y > 0.2 || gamepad1.right_stick_x > 0.2){
                autoTurntoLander = false;
            }

        }

        if (gamepad1.left_stick_button&& !aHeld){
            autoTurntoCrater = true;
            aHeld = true;
        }
        if (!gamepad1.left_stick_button){
            aHeld = false;
        }
        if (gamepad1.left_stick_y > 0.1 && gamepad1.right_stick_x > 0.1){
            autoTurntoCrater = false;
        }
        if (autoTurntoCrater){

            _base.drivetrain.turnTo.goTo(0,0.9);
            if (_base.drivetrain.turnTo.teleopRunSequentially(4, 5)){
                autoTurntoCrater = false;
                _base.drivetrain.stop();
            }

            if (gamepad1.left_stick_y > 0.2 || gamepad1.right_stick_x > 0.2){
                autoTurntoCrater = false;
            }

        }
        if(finalLift)
        {
            telemetry.addLine("LIFT     LIFT    LIFT    LIFT    LIFT ");
            telemetry.addLine("LIFT     LIFT    LIFT    LIFT    LIFT ");
            telemetry.addLine("LIFT     LIFT    LIFT    LIFT    LIFT ");
            telemetry.update();
        }
        else
        {
            telemetry.addData("State: ",_base.drivetrain.state());
            telemetry.addData("State: ",_base.drivetrain.state());
            telemetry.addData("State: ",_base.drivetrain.state());
            telemetry.update();

        }
        //_base.outTelemetry();f
        /*------------------------------------ MARKER DELIVERY --------------------------------*/
        //this is to be used just in case the marker delivery system needs to be used
        if(gamepad2.dpad_right)
        {
            _base.deliver.markerDelivery.setPosition(_base.deliver.markerDelivery.getPosition() + .08);
        }
        else if(gamepad2.dpad_left)
        {
            _base.deliver.markerDelivery.setPosition(_base.deliver.markerDelivery.getPosition() - .08);
        }

        /*---------------------------- HOOK EXTENSION/LIFT ROBOT --------------------------------*/
        if(gamepad2.a || gamepad1.dpad_up)
            _base.latchSystem.extendHook();
        else if(gamepad2.b || gamepad1.dpad_down)
            _base.latchSystem.retractHook();
        else if(gamepad1.x)
            _base.latchSystem.liftRobot();
        else if (gamepad1.y)
            _base.latchSystem.lowerRobot();
        else
            _base.latchSystem.stop();
        /* -------------- HOOK SYSTEM ------------------------*/

        if (gamepad1.dpad_left&& hookOpen){
            _base.latchSystem.closeHook(2100);
            hookOpen = false;
        }
        else if (gamepad1.dpad_right&& !hookOpen){
            _base.latchSystem.openHook(2100);
            hookOpen = true;
        }
        else if (gamepad1.right_trigger > .10)
        {
            _base.latchSystem.openHook();
            hookOpen = true;
        }

        else if (gamepad1.left_trigger > .10)
        {
            _base.latchSystem.closeHook();
            hookOpen = false;
        }
        else{
            _base.latchSystem.stopHook();
        }

        /* -------------- COLLECTING SYSTEM ---------------------*/


        //Precision Based Extension/Retraction
        if(gamepad2.left_stick_y > .15 || gamepad2.left_stick_y < -.15)
        {
            if(gamepad2.left_stick_y > .85)
            {
                _base.collector.powerLift(EXTEND_LIFT_POW);
            }
            else if(gamepad2.left_stick_y < -.85)
            {
                _base.collector.powerLift(-1);
            }
            else
            {
                _base.collector.powerLift(gamepad2.left_stick_y);
            }
        }
        else
            _base.collector.powerLift(0);


        //Collect particles using foam wheel :)
        if(gamepad2.right_trigger > .2)
            if(gamepad2.right_trigger > .65)
                _base.collector.runCollector(1);
            else
                _base.collector.runCollector(gamepad2.right_trigger);
        else if(gamepad2.left_trigger > .2)
            if(gamepad2.left_trigger > .65)
                _base.collector.runCollector(-1);
            else
                _base.collector.runCollector(-gamepad2.left_trigger);
        else if(gamepad2.left_bumper)
        {
            _base.collector.releaseBalls();
        }
        else if(gamepad2.right_bumper)
        {
            _base.collector.runCollector(-.43);
        }
        else
        {
            _base.collector.runCollector(0);
        }



        /*----------------Tilt C-Channel Holding Lifts---------------*/

        if(gamepad2.right_stick_y > .2)
        {
            if(gamepad2.right_stick_y > .85)
                _base.tiltChannel.tiltByPower(1);
            else
                _base.tiltChannel.tiltByPower(gamepad2.right_stick_y);
            automateLift = false;
        }
        else if(gamepad2.right_stick_y < -.2)
        {
            if(gamepad2.right_stick_y < -.85)
                _base.tiltChannel.tiltByPower(-1);
            else
                _base.tiltChannel.tiltByPower(gamepad2.right_stick_y);
            automateLift = false;
        }
        else if (up && automateLift)
        {

            if(_base.tiltChannel.tiltUpByEnc()) {
                automateLift = false;
                up = false;
                down = false;
            }
        }
        else if (down && automateLift)
        {
            if(_base.tiltChannel.tiltDownByEnc())
            {
                automateLift = false;
                up = false;
                down = false;
            }
        }
        else if (!_base.tiltChannel.pulleys.isBusy()) {
            _base.tiltChannel.stop();
        }

        if(gamepad2.y)
        {
            up=true;
            down=false;
            automateLift = true;
        }
        else if(gamepad2.x)
        {
            up=false;
            down=true;
            automateLift = true;
        }



    }

}