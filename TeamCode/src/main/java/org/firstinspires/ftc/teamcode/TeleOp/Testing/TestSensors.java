package org.firstinspires.ftc.teamcode.TeleOp.Testing;

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

        init(hardwareMap, false);
        //initBitmapVuforia(); <-- remove comment if you need to use the camera

        telemetry.addData("teleop:", "waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Write whatever you want to test here

            telemetry.update();
        }

        telemetry.addData("teleop:", "complete");
        telemetry.update();
    }
}
