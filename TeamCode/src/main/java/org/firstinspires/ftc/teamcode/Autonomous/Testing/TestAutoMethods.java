package org.firstinspires.ftc.teamcode.Autonomous.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SkystoneLinearOpMode;
import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name="TestAutoMethods", group = "auto") // BLUE SIDE

public class TestAutoMethods extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap, true);
        //initBitmapVuforia();

        waitForStart();
        launchCycle(3);

    }
}
