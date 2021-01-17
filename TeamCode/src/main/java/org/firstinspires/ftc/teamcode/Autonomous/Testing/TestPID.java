package org.firstinspires.ftc.teamcode.Autonomous.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name="TestPID", group = "auto") // BLUE SIDE
//@Disabled

public class TestPID extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap, -1);
        initOpenCV();

        waitForStart();

        detectStack();

        telemetry.update();
        sleep(10000); // <-- to allow the final telemetry info to show for a couple of seconds before exiting
    }
}
