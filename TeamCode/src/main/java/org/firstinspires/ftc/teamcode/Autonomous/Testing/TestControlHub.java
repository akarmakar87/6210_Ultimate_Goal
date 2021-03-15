package org.firstinspires.ftc.teamcode.Autonomous.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name="TestControlHub", group = "auto") // BLUE SIDE
//@Disabled

public class TestControlHub extends UltimateGoalLinearOpMode {

    int hold;

    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap, -1);
        initOpenCV();

        waitForStart();

        //while (!isStopRequested() && opModeIsActive()){
            hold = detectStack();
        //}

        sleep(1000);

        driveDistance(0.5,5,5);
        telemetry.addData("program done",": complete");
        telemetry.update();

    }
}
