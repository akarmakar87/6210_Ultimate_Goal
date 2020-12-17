package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SkystoneLinearOpMode;
import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name="Test", group = "auto") // BLUE SIDE

public class Test extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize
        init(hardwareMap, true);

        waitForStart();



        telemetry.addData("auto:", "complete");
        telemetry.update();
    }
}
