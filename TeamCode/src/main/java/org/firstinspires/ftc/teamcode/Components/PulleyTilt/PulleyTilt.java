
package org.firstinspires.ftc.teamcode.Components.PulleyTilt;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Core.RobotBase;
import org.firstinspires.ftc.robotcontroller.internal.Core.RobotComponent;
import org.firstinspires.ftc.teamcode.Components.CollectorSystem.CollectorSystem;

/**
 * Created by pmkf2 on 11/11/2018.
 */

public class PulleyTilt extends RobotComponent {

    public DcMotor pulleys;

    public CollectorSystem collector = new CollectorSystem();
    private double TILT_UP_POW = -1;
    private double TILT_DOWN_POW = 1;

    private int TILT_UP_BALL_ENC = -1275;
    private int TILT_UP_BLOCK_ENC = -1800;
    private int TILT_DOWN_ENC = 675;
    public static int CRATER_TILT_TEAM_MARKER_ENC = 700;
    public static int TILT_TEAM_MARKER_ENC = 820;
    private int TILT_AUTO_COLLECT_ENC = 1420;

    public int BUFFER = 75;

    public void init(final RobotBase BASE) {
        super.init(BASE);
        pulleys = mapper.mapMotor("pulleys", DcMotorSimple.Direction.FORWARD);
        collector.init(BASE);
    }

    public void tiltByPower(double POWER) {
        pulleys.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleys.setPower(POWER);
    }

    public boolean tiltUpBallByEnc() {
        //Run without encoder, use custom motion profile
        pulleys.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Pulley position for testing
        base.outTelemetry.addData("Position: ", pulleys.getCurrentPosition());
        base.outTelemetry.update();

        //If already there, stop
        if (pulleys.getCurrentPosition() <= TILT_UP_BALL_ENC) {
            stop();
            return true;
        }
        //Dramatically slow down
        else if (Math.abs(pulleys.getCurrentPosition() - TILT_UP_BALL_ENC) < 300) {
            //TILT_UP_POW *= .95;
            pulleys.setPower(TILT_UP_POW * .95);
        }

        else if (Math.abs(pulleys.getCurrentPosition() - TILT_UP_BALL_ENC) < 600) {
            //TILT_UP_POW *= .95;
            pulleys.setPower(TILT_UP_POW );
            collector.extendCollector.setPower(1);

        }
        else if (Math.abs(pulleys.getCurrentPosition() - TILT_UP_BALL_ENC) < 900) {
            //TILT_UP_POW *= .95;
            pulleys.setPower(TILT_UP_POW );
            collector.extendCollector.setPower(-.70);
        }
        //Set Power to Full
        else {

            pulleys.setPower(TILT_UP_POW);
        }
        return false;
    }

    public boolean tiltUpBlockByEnc() {
        //Run without encoder, use custom motion profile
        pulleys.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Pulley position for testing
        base.outTelemetry.addData("Position: ", pulleys.getCurrentPosition());
        base.outTelemetry.update();

        //If already there, stop
        if (pulleys.getCurrentPosition() <= TILT_UP_BLOCK_ENC) {
            stop();
            return true;
        }
        //Dramatically slow down
        else if (Math.abs(pulleys.getCurrentPosition() - TILT_UP_BLOCK_ENC) < 300) {
            //TILT_UP_POW *= .95;
            pulleys.setPower(TILT_UP_POW * .80);
        }

        //Set Power to Full
        else {
            pulleys.setPower(TILT_UP_POW);
        }
        return false;
    }

    public boolean teleOpTiltDownByEnc() {
        pulleys.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        base.outTelemetry.addData("Position: ", pulleys.getCurrentPosition());
        base.outTelemetry.update();
        if (pulleys.getCurrentPosition() >= TILT_DOWN_ENC) {
            base.outTelemetry.addData("Got Inside the IF Statemnt for TILT UP", true);
            base.outTelemetry.update();
            stop();
            return true;
        } else if (Math.abs(pulleys.getCurrentPosition() - TILT_DOWN_ENC) < 200) {
            //TILT_UP_POW *= .95;
            pulleys.setPower(TILT_DOWN_POW * .35);
        } else if (Math.abs(pulleys.getCurrentPosition() - TILT_DOWN_ENC) < 300) {
            pulleys.setPower(TILT_DOWN_POW * .70);
        } else {
            pulleys.setPower(TILT_DOWN_POW);
        }
        return false;

    }

    public void tiltDownByEnc(int MILLISECONDS) {
        pulleys.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        base.outTelemetry.addData("Position: ", pulleys.getCurrentPosition());
        base.outTelemetry.update();
        long startTime = System.currentTimeMillis();
        while (!(pulleys.getCurrentPosition() >= TILT_DOWN_ENC) && Math.abs(System.currentTimeMillis() - startTime) < MILLISECONDS) {
            if (Math.abs(pulleys.getCurrentPosition() - TILT_DOWN_ENC) < 300)
            {
                pulleys.setPower(TILT_DOWN_POW * .45);
            }
            else if (Math.abs(pulleys.getCurrentPosition() - TILT_DOWN_ENC) < 900)
            {
                pulleys.setPower(TILT_DOWN_POW * .70);
            }
            else
            {
                pulleys.setPower(TILT_DOWN_POW);
            }

        }
        stop();

    }

    public void craterLowestTiltDownByEnc(int MILLISECONDS) {
        pulleys.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        base.outTelemetry.addData("Position: ", pulleys.getCurrentPosition());
        base.outTelemetry.update();
        long startTime = System.currentTimeMillis();
        while (!(pulleys.getCurrentPosition() >= CRATER_TILT_TEAM_MARKER_ENC) && Math.abs(System.currentTimeMillis() - startTime) < MILLISECONDS) {
            if (Math.abs(pulleys.getCurrentPosition() - CRATER_TILT_TEAM_MARKER_ENC) < 300)
            {
                pulleys.setPower(TILT_DOWN_POW * .25);
            }
            else if (Math.abs(pulleys.getCurrentPosition() - CRATER_TILT_TEAM_MARKER_ENC) < 600)
            {
                pulleys.setPower(TILT_DOWN_POW * .80);
            }
            else
            {
                pulleys.setPower(TILT_DOWN_POW);
            }

        }
        stop();

    }
    public void lowestTiltDownByEnc(int MILLISECONDS) {
        pulleys.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        base.outTelemetry.addData("Position: ", pulleys.getCurrentPosition());
        base.outTelemetry.update();
        long startTime = System.currentTimeMillis();
        while (!(pulleys.getCurrentPosition() >= TILT_TEAM_MARKER_ENC) && Math.abs(System.currentTimeMillis() - startTime) < MILLISECONDS) {
            if (Math.abs(pulleys.getCurrentPosition() - TILT_TEAM_MARKER_ENC) < 300)
            {
                pulleys.setPower(TILT_DOWN_POW * .25);
            }
            else if (Math.abs(pulleys.getCurrentPosition() - TILT_TEAM_MARKER_ENC) < 600)
            {
                pulleys.setPower(TILT_DOWN_POW * .80);
            }
            else
            {
                pulleys.setPower(TILT_DOWN_POW);
            }

        }
        stop();

    }

    public void AUTOTiltToZero(int MILLISECONDS) {
        pulleys.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        base.outTelemetry.addData("Position: ", pulleys.getCurrentPosition());
        base.outTelemetry.update();
        long startTime = System.currentTimeMillis();
        while (!(pulleys.getCurrentPosition() <= 15) && Math.abs(System.currentTimeMillis() - startTime) < MILLISECONDS) {
            if (Math.abs(pulleys.getCurrentPosition()) <= 300)
            {
                pulleys.setPower(TILT_UP_POW * .75);
            }
            else if (Math.abs(pulleys.getCurrentPosition()) < 600)
            {
                pulleys.setPower(TILT_UP_POW * 1);
            }
            else
            {
                pulleys.setPower(TILT_UP_POW);
            }

        }
        stop();

    }




    public void AutoTiltToCollect(int TIMEOUT) {
        pulleys.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        base.outTelemetry.addData("Position: ", pulleys.getCurrentPosition());
        base.outTelemetry.update();
        long startTime = System.currentTimeMillis();
        while (!(pulleys.getCurrentPosition() >= TILT_TEAM_MARKER_ENC) && Math.abs(System.currentTimeMillis() - startTime) < TIMEOUT) {
            if (Math.abs(pulleys.getCurrentPosition() - TILT_TEAM_MARKER_ENC) < 300)
            {
                pulleys.setPower(TILT_AUTO_COLLECT_ENC * .25);
            }
            else if (Math.abs(pulleys.getCurrentPosition() - TILT_TEAM_MARKER_ENC) < 600)
            {
                pulleys.setPower(TILT_AUTO_COLLECT_ENC * .70);
            }
            else
            {
                pulleys.setPower(TILT_AUTO_COLLECT_ENC);
            }

        }
        stop();

    }


    public boolean tiltToEncoder (int encoders){
        pulleys.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        base.outTelemetry.addData("Position: ", pulleys.getCurrentPosition());
        base.outTelemetry.update();
        if (pulleys.getCurrentPosition() >= encoders )
        {
            base.outTelemetry.addData("Got Inside the IF Statemnt for TILT UP", true);
            base.outTelemetry.update();
            stop();
            return true;
        }
        else if(Math.abs(pulleys.getCurrentPosition() - encoders) < 900)
        {
            //TILT_UP_POW *= .95;
            pulleys.setPower(TILT_DOWN_POW * .85);
        }
        else if(Math.abs(pulleys.getCurrentPosition() - encoders) < 300)
        {
            pulleys.setPower(TILT_DOWN_POW * .60);
        }
        else {
            pulleys.setPower(TILT_DOWN_POW);
        }
        return false;
    }

    public void outTelemetry()
    {
        base.outTelemetry.write("-----------PULLEYS-----------------");
        base.outTelemetry.addData("Pulley Power: ", pulleys.getPower());
        base.outTelemetry.addData("Pulley Position: ", pulleys.getCurrentPosition());
        base.outTelemetry.write("----------END PULLEYS-----------");
        base.outTelemetry.newLine();
    }

    @Override
    public void stop()
    {
        pulleys.setPower(0);
    }
}
