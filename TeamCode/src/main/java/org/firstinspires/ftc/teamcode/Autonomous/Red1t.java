package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name="Red 1t", group = "auto")

public class Red1t extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //IMPORTANT THINGS TO NOTE
        //1) There are two wobble in auto. Wobble 1 is preloaded in the robot while Wobble 2 is in the next position over

        //2) "Designated Depot" means the depot where the wobbles are supposed to be places. (Known through ring detection)

        //3) "Central position" is the front edge of the first square tile that the position 1 starting position is located in.
        //It is called so because it is where all the following movements are based from. It was chosen for its location away
        //from any turning obstacles and it's alignment with Wobble 2. When moving to "Central position" the rear of the robot
        //should be at the position rather than the middle or front of the robot. This is because if the robot turns 90
        //degrees on that position the wobble arm will directly face Wobble 2.

        //sets up imu and vuforia
        init(hardwareMap, 1);
        initOpenCV();

        int pos, dist = 0;
        int angle = 0;
        waitForStart();

        // Detect rings
        pos = detectStack();
        strafeAdjust(1, 12, 0, 3000);

        // Set values based on # of rings
        switch (pos) {
            case 0:
                dist = 60;
                angle = -15;
                break;
            case 1:
                dist = 90;
                angle = 3;
                break;
            case 4:
                dist = 120;
                angle = -5;
                break;
        }

        // Enter central position
        driveAdjust(0.8, 24, 0, 5000);

        //Drive to designated depot
        turnPID(angle,0.8/180,0.0001,0.5,5000);
        driveAdjust(1, dist, angle, 5000);

        // Release wobble
        setWobbleArm(true);
        setWobbleClaw(false);
        setWobbleArm(false);

        // Return to central position
        driveAdjustShooter(0.8, dist, angle, 5000, 1);

        // Rotate to fire
        turnPID(180,0.8/180,0.00005,0.1,5000);

        // Fire rings at powershots
        turnPID(-178,0.6/180,0.00005,0.1,1000);
        sleep(1000);
        setLoader(true);
        setLoader(false);
        turnPID(-176,0.6/180,0.00005,0.1,1000);
        setLoader(true);
        setLoader(false);
        turnPID(-174,0.6/180,0.00005,0.1,1000);
        setLoader(true);
        setLoader(false);
        shooter.setPower(0);

        // Retrieve wobble 2
        turnPID(90,0.6/180,0.00005,0.1,1000);
        setWobbleArm(true);
        driveAdjust(0.8, 15, 180, 2000);
        setWobbleClaw(true);
        setWobbleArm(false);
        driveAdjust(-0.8, 15, 180, 2000);

        //Drive to designated depot
        turnPID(angle,0.8/180,0.0001,0.5,5000);
        driveAdjust(1, dist, angle, 5000);

        // Release wobble 2
        setWobbleArm(true);
        setWobbleClaw(false);
        setWobbleArm(false);

        // Park
        //There is no case for position 1: it is assumed the robot will place the wobble while on the navigation line
        switch (pos) {
            case 0:
                driveAdjust(-0.8, 10, angle, 2000);
                turnPID(0,0.8/180,0.0001,0.5,5000);
                driveAdjust(0.8, 20, 0, 2000);
                break;
            case 4:
                turnPID(0,0.8/180,0.0001,0.5,5000);
                driveAdjust(-0.8, 50, 0, 2000);
                break;
        }

        telemetry.addData("auto:", "complete");
        telemetry.update();
    }
}

