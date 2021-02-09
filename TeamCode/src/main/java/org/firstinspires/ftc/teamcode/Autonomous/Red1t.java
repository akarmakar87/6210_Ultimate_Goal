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
        //strafeAdjust(1, 12, 0, 3000);

        // Set values based on # of rings
        switch (pos) {
            case 0:
                dist = 38;
                angle = -30;
                break;
            case 1:
                dist = 65;
                angle = 0;
                break;
            case 4:
                dist = 96;
                angle = -15;
                break;
        }

        // Enter central position
        driveAdjust(1, 18, 0, 5000);

        //Drive to designated depot
        turnPID(angle,0.8/180,0.0001,0.5,5000);
        driveAdjust(0.9, dist+2, angle, 5000);

        // Release wobble
        setWobbleArm(true);
        setWobbleClaw(false);
        setWobbleArm(false);

        // Return to central position
        driveAdjustShooter(-1, dist-6, angle, 5000, 0.95);
        setWobbleClaw(true);

        // Fire rings at powershots
        turnPID(170,0.6/180,0.00005,0.1,3000);
        wobbleArm.setPower(0);
        sleep(1000);
        setLoader(true);
        setLoader(false);
        sleep(1000);
        //turnPID(-178,0.4/180,0.00000,0.1,3000);
        setLoader(true);
        setLoader(false);
        sleep(1000);
        //turnPID(-176,0.4/180,0.00000,0.1,3000);
        setLoader(true);
        setLoader(false);
        shooter.setPower(0);
        wobbleArm.setPower(1);

        //driveAdjust(-1, 40, 180, 5000);

        // Retrieve wobble 2
        setWobbleClaw(false);
        turnPID(115,0.8/180,0.00005,0.1,2500);
        setWobbleArm(true);
        driveAdjust(0.5, 4, 115, 2000);
        setWobbleClaw(true);
        sleep(750);
        setWobbleArm(false);        //This may not be necessary if we encounter issues bringing the wobble back into the robot. We could just drag and push the wobble.
        sleep(1000);
        driveAdjust(-0.5, 4, 115, 2000);

        //Drive to designated depot
        turnPID(angle+5,0.8/180,0.0001,0.5,5000);
        driveAdjust(1, dist-10, angle+5, 5000);

        // Release wobble 2
        setWobbleArm(true);
        setWobbleClaw(false);
        setWobbleArm(false);
        sleep(250);

        // Park
        //There is no case for position 1: it is assumed the robot will place the wobble while on the navigation line
        switch (pos) {
            case 0:
                driveAdjust(-1, 10, 0, 2000);
                turnPID(35,1/180,0.0001,0.5,2500);
                driveAdjust(1, 30, 10, 2000);
                break;
            case 4:
                turnPID(0,0.8/180,0.0001,0.5,2000);
                driveAdjust(-1, 50, 0, 2500);
                break;
        }

        telemetry.addData("auto: ", "complete");
        telemetry.update();
    }
}

