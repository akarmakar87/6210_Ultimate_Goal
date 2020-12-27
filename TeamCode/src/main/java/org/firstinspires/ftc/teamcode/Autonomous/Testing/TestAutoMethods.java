package org.firstinspires.ftc.teamcode.Autonomous.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SkystoneLinearOpMode;
import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name="TestAutoMethods", group = "auto") // BLUE SIDE
//@Disabled

public class TestAutoMethods extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap, true);
        // initBitmapVuforia(); <-- remove comment if you need to use the camera

        waitForStart();

        // write the methods that you want to test here
        driveAdjust(0.6, 48, 180, 5000);
        //turnPID(180,0.8/180,0.0001,0.5,5000);
        telemetry.update();
        sleep(10000); // <-- to allow the final telemetry info to show for a couple of seconds before exiting
    }
}
