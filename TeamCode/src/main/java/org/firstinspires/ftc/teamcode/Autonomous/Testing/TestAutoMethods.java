package org.firstinspires.ftc.teamcode.Autonomous.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name="TestAutoMethods", group = "auto") // BLUE SIDE
//@Disabled

public class TestAutoMethods extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap, 1);
        //int pos = -5;
        //initOpenCV();
        //initBitmapVuforia();// <-- remove comment if you need to use the camera

        waitForStart();

       intake.setPower(-1);
       sleep(2000);
       driveAdjust(1, 12, 0, 3000);
       sleep (3000);
       driveAdjust(1, 10, 0, 3000);
       sleep (3000);


       setWobbleArm(true);


       sleep(3000);


    }
}
