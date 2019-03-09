// This autonomous is intended for an initial setup where the robot is hanging from the lander facing the crater
// The robot will detach and lower itself and then locate the position of the block using the phone
// The robot will knock the block off and then drive to our teammate's side, where it will knock their block off also
// The robot will then deposit our team marker and park in the crater in was initially facing

package org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.Core.Utility.CustomTensorFlow;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Components.PulleyTilt.PulleyTilt;
import org.firstinspires.ftc.teamcode.SeasonCode.RoverRuckus.Base;

@Autonomous(name = "Park Double Crater")

// the name of the class is misleading, refer to the Autonomous name
//this is the main double crater auto
public class ParkDoubleCrater extends LinearOpMode {

    private Base _base = new Base();
    //private UtilGoldDetector eye;
    private CustomTensorFlow detector;


    private boolean runUsingTensorFlow = true; // if not, then running with open cv is assumed

    private blockState _block;

    private final static double FAR_PARTICLE_ANGLE = 31;
    private final static double MIDDLE_ANGLE = 10;
    private final static double SECOND_BLOCK_ABORT_ANGLE = 315;
    private final static double MARKER_ANGLE = 184;

    private final static double TURN_INCREMENT = 5;

    private final static double TURN_SPEED = 0.33;
    private final static double BLOCK_TURN_SPEED = 0.55;
    private final static double DRIVING_SPEED = 0.63;
    private final static double DRIVING_SPEED_CRATER = .95;
    private final static double DRIVING_SPEED_BLOCK = .53;
    public double FINAL_CONFIDENCE = 0;
    public double silverConfidence = 0;

    // these are the only final values that are used multiple times
    private double block_distance = 26;
    private final static double SECOND_BLOCK_DISTANCE = 23.0;

    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final double ACCEPTABLE_CONFIDENCE = 0.45;

    //Hold state of where gold block is sitting
    private enum blockState
    {
        LEFT, MIDDLE, RIGHT, UNCERTAIN
    }

    @Override
    public void runOpMode()
    {
        _base.init(hardwareMap, this);
        _base.outTelemetry.write("Initializing - DO NOT START UNTIL NEXT MESSAGE");
        _base.outTelemetry.update();
        _block = blockState.UNCERTAIN;
        detector = new CustomTensorFlow(hardwareMap);
        runUsingTensorFlow = true;
        //eye = new UtilGoldDetector(hardwareMap);
        //This calibration is done before landing because the landing could "bump" the robot and change our angle
        _base.outTelemetry.write("All Systems Go");
        _base.outTelemetry.update();


        while (! opModeIsActive()){
            telemetry.addData("Waiting to start", "no crashing");
            telemetry.update();
        }

        //Gets the robot onto the field from the hanger
        _base.latchSystem.lowerRobot(3250);

        _base.drivetrain.driveTo.goTo(0.25,0.1);
        _base.drivetrain.driveTo.runSequentially();

        _base.imu.calibrateTo(0);

        _base.latchSystem.extendHook(0);
        _base.latchSystem.openHook(1300);
        telemetry.addData("communicate with rev hub", "so doesn't disconnect");
        _base.latchSystem.openHook(1000);


        //makes sure the landing did not get our robot off course by turning to the angle that we initialized our gyroscope to
//        _base.drivetrain.turnTo.goTo(1,.20);
//        _base.drivetrain.turnTo.blockRunSequentially(3, 2.5);

        //drives forward to avoid hitting the lander while turning
        _base.drivetrain.driveTo.goTo(4,DRIVING_SPEED/2);
        _base.drivetrain.driveTo.runSequentially();

//        _base.drivetrain.turnTo.goTo(1, .35);
//        _base.drivetrain.turnTo.runSequentially();

        detector.activate();
        _base.drivetrain.driveTo.goTo(3,DRIVING_SPEED/2);
        _base.drivetrain.driveTo.runSequentially();
        this.sleep(800);
        if(_block == blockState.UNCERTAIN)
        {
            if (relativelyAligned()) {
                _block = blockState.MIDDLE;
                telemetry.addData("FIRST MIDDLE CHECK FOUND", "");
                telemetry.update();
            }
            // pans across the particles until it either sees the block or reaches 348 degrees
            // 348 degrees should be past the far right particle
        }


        // method that sends information about our angles and powers
        //sendTelemetry();

        //turns to the far right in preparation for panning across the particles from right to left
        if(_block == blockState.UNCERTAIN)
        {
            //to use one run of aligned to make sure stuff works
            _base.drivetrain.turnTo.goTo(330,BLOCK_TURN_SPEED-.2);
            _base.drivetrain.turnTo.blockRunSequentially(3,5);

            this.sleep(700);
            if (aligned()) {
                _block = blockState.RIGHT;
                telemetry.addData("FOUND IN RIGHT", "");
                telemetry.update();
            }
            // pans across the particles until it either sees the block or reaches 348 degrees
            // 348 degrees should be past the far right particle
        }
        if (_block == blockState.UNCERTAIN){
            for  (double i = 334; i < 337; i += TURN_INCREMENT - 1){
                telemetry.addData("Searching for right block!" , "");
                telemetry.update();
                _base.drivetrain.turnTo.goTo(i,BLOCK_TURN_SPEED-.2);
                _base.drivetrain.turnTo.blockRunSequentially();
                if (aligned()){
                    _block = blockState.RIGHT;
                    break;
                }
            }
        }
        // if it is not in the middle, the robot turns until it sees the left block or reaches 18 degrees

//        if(_block == blockState.UNCERTAIN) {
//            _base.drivetrain.turnTo.goTo(2, BLOCK_TURN_SPEED - .2);
//            _base.drivetrain.turnTo.blockRunSequentially();
//
//            this.sleep(1000);
//            if (aligned()) {
//                _block = blockState.MIDDLE;
//                telemetry.addData("FOUND IN MIDDLE", "");
//                telemetry.update();
//            }
//        }
        //if the block is still not found, the block is on the left
        // the robot turns until it reaches 17 degrees and then pans until it sees the block

        if(_block == blockState.UNCERTAIN)
        {
            _base.drivetrain.turnTo.goTo( 37, TURN_SPEED+.1);
            _base.drivetrain.turnTo.blockRunSequentially(2,5);
            _block = blockState.LEFT;
        }



        //this turns a small amount to account for the offset of our phone on the left side of our robot


        //sendTelemetry();
        telemetry.addData("block state is", _block);
        telemetry.addData("Silver Confidence = ", silverConfidence);
        telemetry.addData("Gold Confidence = ", FINAL_CONFIDENCE);
        telemetry.update();

        //drive forward to knock the block off and then go back the same distance
        // this works because at this point the robot is facing the block
        if (_block == blockState.MIDDLE){
            block_distance -= 4;
        }
        else if(_block == blockState.RIGHT)
        {
            //block_distance -= 1.0;
        }
        else
        {
            //block_distance -= 1;
        }

        _base.drivetrain.driveTo.goTo(block_distance - 1,DRIVING_SPEED_BLOCK);
        _base.drivetrain.driveTo.runSequentially();

        if(_block == blockState.RIGHT)
        {
            _base.drivetrain.driveTo.goTo(-(block_distance),DRIVING_SPEED_BLOCK);
            _base.drivetrain.driveTo.runSequentially();
        }
        else if(_block == blockState.MIDDLE)
        {
            _base.drivetrain.driveTo.goTo(-(block_distance-1), DRIVING_SPEED_BLOCK);
            _base.drivetrain.driveTo.runSequentially();
        }
        else
        {
            _base.drivetrain.driveTo.goTo(-(block_distance - 4), DRIVING_SPEED_BLOCK);
            _base.drivetrain.driveTo.runSequentially();
        }

        // the robot is at a common spot, but a different angle based on where the block was
        // since the turnTo class uses a gyroscope, turning to 60 gives a common angle also
        if(_block == blockState.LEFT)
        {

            _base.drivetrain.turnTo.goTo(63, TURN_SPEED-.05);
            _base.drivetrain.turnTo.runSequentially(2,5);

        }
        else if(_block == blockState.RIGHT)
        {

            _base.drivetrain.turnTo.goTo(68.5, TURN_SPEED-.05);
            _base.drivetrain.turnTo.runSequentially(2,5);

        }
        else
        {

            _base.drivetrain.turnTo.goTo(65, TURN_SPEED-0.1);
            _base.drivetrain.turnTo.runSequentially(2,5);

        }

        // drives between the lander and the far left particle so the path is clear to our teammate's side
        if (_block == blockState.RIGHT){
            _base.drivetrain.driveTo.goTo(40, DRIVING_SPEED);
            _base.drivetrain.driveTo.runStopIfDist(17, 3);

            _base.drivetrain.turnTo.goTo(135, TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially();

            _base.drivetrain.driveTo.goTo(36, DRIVING_SPEED);
            _base.drivetrain.driveTo.runSequentially();

            _base.drivetrain.turnTo.goTo(110, TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially();

            _base.drivetrain.driveTo.goTo(4, DRIVING_SPEED);
            _base.drivetrain.driveTo.runSequentially();

            deliver();

            _base.drivetrain.driveTo.goTo(10, DRIVING_SPEED);
            _base.drivetrain.driveTo.runSequentially();

            _base.drivetrain.turnTo.goTo(360 - 33, TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially();

            _base.drivetrain.driveTo.goTo(40, 0.9);
            _base.drivetrain.driveTo.runSequentially();

        }

        else if (_block== blockState.MIDDLE){



            _base.drivetrain.driveTo.goTo(60, DRIVING_SPEED);
            _base.drivetrain.driveTo.runStopIfTouch(8);

            //turn to drive in between particle on teammate's side and wall
            _base.drivetrain.turnTo.goTo(126, TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially(2);


            //drives between the particle on teammate's side and wall
            _base.drivetrain.driveTo.goTo(11, DRIVING_SPEED );
            _base.drivetrain.driveTo.runSequentially(10);



            // turns in preparation for moving towards the deposit zone
            _base.drivetrain.turnTo.goTo(134, TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially(3);

            _base.tiltChannel.pulleys.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            _base.tiltChannel.pulleys.setTargetPosition(PulleyTilt.TILT_TEAM_MARKER_ENC);
            _base.tiltChannel.pulleys.setPower(1);

            //drives to the deposit zone
            _base.drivetrain.driveTo.goTo(45, DRIVING_SPEED);
            _base.drivetrain.driveTo.runStopIfDist(7);

            deliver();

            _base.drivetrain.turnTo.goTo(143, TURN_SPEED );
            _base.drivetrain.turnTo.arcSequentially(3, 2.5);


            _base.drivetrain.turnTo.goTo(144, TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially(2,1);

            _base.drivetrain.driveTo.goTo(18, 0.5);
            _base.drivetrain.driveTo.runSequentially();

            _base.drivetrain.turnTo.goTo(260, BLOCK_TURN_SPEED-.1);
            _base.drivetrain.turnTo.runSequentially(2,4);

            //knock block off
            _base.drivetrain.driveTo.goTo(27, DRIVING_SPEED);
            _base.drivetrain.driveTo.runSequentially();

            //back up into the depot
            _base.drivetrain.driveTo.goTo(-30, DRIVING_SPEED);
            _base.drivetrain.driveTo.runSequentially();

            _base.drivetrain.turnTo.goTo(360 -40, TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially();

            _base.drivetrain.driveTo.goTo(32, DRIVING_SPEED);
            _base.drivetrain.driveTo.runSequentially();

            _base.drivetrain.turnTo.goTo(360-42, TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially();

            _base.drivetrain.driveTo.goTo(38, 1);
            _base.drivetrain.driveTo.runSequentially();
        }

        else{
            //Block State: left

            _base.drivetrain.driveTo.goTo(60, DRIVING_SPEED);
            _base.drivetrain.driveTo.runStopIfTouch(8);

            //turn to drive in between particle on teammate's side and wall
            _base.drivetrain.turnTo.goTo(126, TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially(2);


            //drives between the particle on teammate's side and wall
            _base.drivetrain.driveTo.goTo(11, DRIVING_SPEED );
            _base.drivetrain.driveTo.runSequentially(10);



            // turns in preparation for moving towards the deposit zone
            _base.drivetrain.turnTo.goTo(134, TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially(3);


            //drives to the deposit zone
            _base.drivetrain.driveTo.goTo(45, DRIVING_SPEED);
            _base.drivetrain.driveTo.runStopIfDist(7);

            deliver();

            _base.drivetrain.turnTo.goTo(143, TURN_SPEED );
            _base.drivetrain.turnTo.arcSequentially(3, 2.5);


            _base.drivetrain.turnTo.goTo(144, TURN_SPEED);
            _base.drivetrain.turnTo.runSequentially(2,1);

            _base.drivetrain.driveTo.goTo(8.5, 0.5);
            _base.drivetrain.driveTo.runSequentially();

            _base.drivetrain.turnTo.goTo(207, BLOCK_TURN_SPEED-.1);
            _base.drivetrain.turnTo.runSequentially();

            _base.drivetrain.driveTo.goTo(65, 0.9);
            _base.drivetrain.driveTo.runSequentially();
        }


        _base.drivetrain.stop();
        detector.deactivate();
        while(opModeIsActive()) {
            _base.collector.powerExtension(0);
        }
    }

    private void sendTelemetry(){
        telemetry.addData("Angle Z: ", _base.imu.zAngle());
        telemetry.addData("Angle X: ", _base.imu.xAngle());
        telemetry.addData("Angle Y: ", _base.imu.yAngle());
        telemetry.addData("SPEED: ", _base.drivetrain.frontLeft().getPower());
        telemetry.update();
    }

    private void deliver(){
        _base.collector.powerExtension(-1);
        try{
            Thread.sleep(200);}
        catch(Exception ex){ex.printStackTrace();}

        _base.tiltChannel.lowestTiltDownByEnc(3000);
        _base.collector.runCollector(-.40);

        // gives time for the marker to slide off
        try{
            Thread.sleep(750);}
        catch(Exception ex){ex.printStackTrace();}
        _base.collector.powerExtension(0);
        _base.collector.runCollector(-.25);
        _base.tiltChannel.AUTOTiltToZero(3500);
        //_base.collector.powerExtension(-.65);
//        try{
//            Thread.sleep(800);}
//        catch(Exception ex){ex.printStackTrace();}
        _base.collector.powerExtension(0);
        _base.collector.stop();
    }

    private void testDeliver(){
        _base.collector.powerExtension(-1);
        try{
            Thread.sleep(200);}
        catch(Exception ex){ex.printStackTrace();}

        _base.tiltChannel.lowestTiltDownByEnc(3000);
        _base.collector.runCollector(-.40);

        // gives time for the marker to slide off
        try{
            Thread.sleep(750);}
        catch(Exception ex){ex.printStackTrace();}
        _base.collector.powerExtension(0);
        _base.collector.runCollector(-.25);
        _base.tiltChannel.AUTOTiltToZero(3500);
        //_base.collector.powerExtension(-.65);
//        try{
//            Thread.sleep(800);}
//        catch(Exception ex){ex.printStackTrace();}
        _base.collector.powerExtension(0);
        _base.collector.stop();
    }

    private void turnToAlign(){
        if(_block == blockState.LEFT)
        {
            _base.drivetrain.turnTo.goTo(_base.imu.zAngle() + 18, TURN_SPEED);
            _base.drivetrain.turnTo.blockRunSequentially();
            telemetry.addData("Tur to Align LEFT,", "");
            telemetry.update();
        }
        else if (_block == blockState.RIGHT)
        {
            _base.drivetrain.turnTo.goTo(_base.imu.zAngle() + 2, TURN_SPEED);
            _base.drivetrain.turnTo.blockRunSequentially();
            telemetry.addData("Tur to Align RIGHT,", "");
            telemetry.update();
        }
        else {
            _base.drivetrain.turnTo.goTo(_base.imu.zAngle() + 12, TURN_SPEED);
            _base.drivetrain.turnTo.blockRunSequentially();
            telemetry.addData("Tur to Align MIDDLE,", "");
            telemetry.update();
        }
    }

    private void turnToAlign(double ANGLE_ADDITION){
        _base.drivetrain.turnTo.goTo(_base.imu.zAngle() + ANGLE_ADDITION, TURN_SPEED);
        _base.drivetrain.turnTo.blockRunSequentially();
    }
    private void driveAndExtend(double distance, double timeout){
        double COUNTS_PER_INCH = 40.78651685 * .80;
        timeout *= 1000;
        if(_base.drivetrain.getEncoderMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            _base.drivetrain.encoderToPos();
        }

        _base.drivetrain.backLeft().setTargetPosition((int)(distance * COUNTS_PER_INCH) + _base.drivetrain.backLeft().getCurrentPosition());
        _base.drivetrain.backRight().setTargetPosition((int)(distance * COUNTS_PER_INCH)+ _base.drivetrain.backRight().getCurrentPosition());
        _base.drivetrain.frontLeft().setTargetPosition((int)(distance * COUNTS_PER_INCH) + _base.drivetrain.frontLeft().getCurrentPosition());
        _base.drivetrain.frontRight().setTargetPosition((int)(distance * COUNTS_PER_INCH) + _base.drivetrain.frontRight().getCurrentPosition());

        _base.drivetrain.setAllMotorPower(1);
        long startTime = System.currentTimeMillis();

        while((_base.drivetrain.isBusy()  && opModeIsActive() && System.currentTimeMillis() - startTime < timeout))
        {
            _base.collector.powerExtension(-1);
        }

        _base.collector.powerExtension(0);
        _base.drivetrain.setAllMotorPower(0);

    }

    public boolean aligned(){
        boolean aligned = false;
        double maxSilver = 0;
        double maxGold = 0;
        if (runUsingTensorFlow){
            detector.refresh();
            if(detector.recognitions == null)
            {
                aligned = false;
            }
            else{
                for (int i = 0; i < detector.recognitions.size(); i ++){
                    Recognition rec = detector.recognitions.get(i);
                    if (rec.getLabel().equals(LABEL_SILVER_MINERAL)){
                        telemetry.addData("Silver detecting with confidence ", rec.getConfidence());
                        silverConfidence = rec.getConfidence();
                        if(rec.getConfidence() > 0.85) {
                            silverConfidence = rec.getConfidence();
                            return false;
                        }
                        break;
                    }
                    if (rec.getLabel().equals(LABEL_GOLD_MINERAL) && rec.getConfidence() > ACCEPTABLE_CONFIDENCE){
                        telemetry.addData("Gold detecting with confidence ", rec.getConfidence());
                        FINAL_CONFIDENCE = rec.getConfidence();
                        return true;
                    }
                }
            }

        }
        else{
            //aligned = eye.isAligned();
        }
        telemetry.addData("END RESULT - ", aligned);
        telemetry.update();
        sleep(500);
        return aligned;

    }

    public boolean relativelyAligned(){

        if (runUsingTensorFlow){
            double silverConfidence = 0;
            double goldConfidence = 0;
            detector.refresh();
            if(detector.recognitions == null)
            {
                return false;
            }
            else{
                for (int i = 0; i < detector.recognitions.size(); i ++){
                    Recognition rec = detector.recognitions.get(i);

                    if (rec.getLabel().equals(LABEL_SILVER_MINERAL)){
                        telemetry.addData("Silver detecting with confidence ", rec.getConfidence());
                        telemetry.update();
                        if (rec.getConfidence() > silverConfidence){
                            silverConfidence = rec.getConfidence();
                        }
                    }
                    if (rec.getLabel().equals(LABEL_GOLD_MINERAL) && rec.getConfidence() > ACCEPTABLE_CONFIDENCE){
                        telemetry.addData("Gold detecting with confidence ", rec.getConfidence());
                        telemetry.update();
                        if (rec.getConfidence() > goldConfidence){
                            goldConfidence = rec.getConfidence();
                        }
                    }
                }
            }
            if (goldConfidence > silverConfidence && goldConfidence > ACCEPTABLE_CONFIDENCE){
                return true;
            }
            else{
                return false;
            }

        }
        else{
            return false;
        }
    }

    private boolean isAligned() {
        detector.refresh();

        if (detector.recognitions == null || detector.recognitions.size() == 0) {
            return false;
        }
        else if (detector.recognitions.size() == 1 && detector.recognitions.get(0).getLabel() == LABEL_GOLD_MINERAL) {
            if (detector.recognitions.get(0).getConfidence() > 0.35) {
                return true;
            } else {
                return false;
            }
        }
        else {
            double minHeight;
            double maxHeight;
            double heightDifference = 90;

            maxHeight = detector.recognitions.get(0).getLeft();
            minHeight = detector.recognitions.get(0).getLeft();
            for (Recognition r : detector.recognitions) {
                if (r.getLeft() > maxHeight) {
                    maxHeight = r.getLeft();
                }
                if (r.getLeft() < minHeight) {
                    minHeight = r.getLeft();
                }
            }
            if ((maxHeight - minHeight) > heightDifference) {
                for (Recognition r : detector.recognitions) {
                    if (r.getLeft() < (maxHeight - heightDifference)) {
                        detector.recognitions.remove(r);
                    }
                }
            }


            if (detector.recognitions.size() > 1) {

                double center = detector.recognitions.get(0).getImageHeight() / 2;

                double minOffset = 2000;

                for (Recognition r : detector.recognitions) {
                    double offset = Math.abs((r.getTop() - r.getBottom()) - center);
                    if (offset < minOffset) {
                        minOffset = offset;
                    }
                }

                for (Recognition r : detector.recognitions) {
                    if (Math.abs((r.getTop() - r.getBottom()) - center) > minOffset) {
                        detector.recognitions.remove(r);
                    }
                }

            }

        }
        if (detector.recognitions.size() > 1){
            return false;
        }
        else if (detector.recognitions.get(0).getLabel() == LABEL_GOLD_MINERAL && detector.recognitions.get(0).getConfidence() > 0.35) {
            return true;
        }
        else {
            return false;
        }

    }
}







