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

       // turnPID(15,0.7/180,0.0,0.20,5000);
        //telemetry.addData("angle", get180Yaw());


        shooter.setPower(1);
        sleep(2000);
        setLoader(true);
        setLoader(false);
        setLoader(true);
        setLoader(false);
        //sleep(500);
        setLoader(true);
        setLoader(false);

        // write the methods that you want to test here
        //driveAdjustShooter(0.8, 45, 0, 5000, 1);

        /*startShooter(1);
        sleep(2000);
        shooter.setPower(1);
        //Turn to aim

        setLoader(true);
        setLoader(false);
        sleep(750);
        //Turn to aim

        setLoader(true);
        setLoader(false);
        sleep(750);
        //Turn to aim

        setLoader(true);
        setLoader(false);
        shooter.setPower(0);

        //Turn to original position

        //turnPID(180,0.8/180,0.0001,0.5,5000);
        telemetry.update();
        sleep(10000); // <-- to allow the final telemetry info to show for a couple of seconds before exiting
         */

    }
}
