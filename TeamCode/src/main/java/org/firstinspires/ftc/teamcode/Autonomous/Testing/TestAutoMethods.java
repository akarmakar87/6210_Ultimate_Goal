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

        telemetry.update();
        sleep(2000); // <-- to allow the final telemetry info to show for a couple of seconds before exiting
    }
}
