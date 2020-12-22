package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

import static android.graphics.Color.green;
import static android.graphics.Color.red;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TestSensors", group = "teleop")
//@Disabled
public class TestSensors extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        init(hardwareMap, true);
        initBitmapVuforia();

        telemetry.addData("teleop:", "waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            /*if (Math.abs(gamepad1.left_stick_x) > 0.05){
                setMotorPowers("STRAFE", gamepad1.left_stick_x,0,0,0);
            }else{
                setMotorPowers("ALL",0,0,0,0);
            }*/

           /* Bitmap b = getBitmap();

            //telemetry.addData("origin pixel:", green(b.getPixel(600,360))); // 600 --> 800 is good top threshold, a bit close tho
            detectStack(b,true);
            telemetry.update();*/


        }

        telemetry.addData("teleop:", "complete");
        telemetry.update();
    }
}
