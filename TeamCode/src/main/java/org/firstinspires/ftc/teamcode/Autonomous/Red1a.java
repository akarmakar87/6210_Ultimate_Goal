package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name="Red 1a", group = "auto") // RED SIDE

public class Red1a extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //sets up imu and vuforia
        init(hardwareMap, 1);
        initOpenCV();

        int pos, dist = 0;
        int wait = 0;
        double adjust = 0.0;
        double longAdjust = 0.0;

        //CHOSE WHETHER SHOOTING FOR HIGH GOAL OR POWERSHOT (BUTTON ON GAMEPAD) : NOT INCLUDED

        waitForStart();

        // DETECT # OF RINGS
        pos = detectStack();
        strafeAdjust(1, 9, 0, 3000);

        // ALIGN WITH DEPOT
        switch (pos) {
            case 0:
                dist = 55;
                wait = 1250;
                break;
            case 1:
                dist = 85;
                wait = 750;
                break;
            case 4:
                dist = 100;
                wait = 500;
                break;
        }

        driveAdjust(1, dist, 0, 5000);

        // TURN TOWARD DEPOT
        if (pos == 0 || pos == 4)
            turnPID(-45,0.8/180,0.0001,0.5,5000);
        else
            turnPID(45,0.8/180,0.0001,0.5,5000);

        // RELEASE WOBBLE
        //deploy arm
        setWobbleArm(true);
        //release wobble
        setWobbleClaw(false);
        //retract arm
        setWobbleArm(false);

        // ALIGN PERPENDICULAR TO WHITE LINE
        turnPID(180,0.8/180,0.00005,0.1,5000);

        // BACK UP PAST WHITE LINE
        driveAdjustShooter(0.8, dist-14, 180, 5000, 1);

        // ROTATE TO FIRE AT RIGHT POWERSHOT
        // turnPID(180,0.6/180,0.00005,0.1,5000);

        // FIRE 1
        turnPID(-178,0.6/180,0.00005,0.1,1000);
        setLoader(true);
        setLoader(false);
        turnPID(-176,0.6/180,0.00005,0.1,1000);
        setLoader(true);
        setLoader(false);
        turnPID(-174,0.6/180,0.00005,0.1,1000);
        setLoader(true);
        setLoader(false);
        shooter.setPower(0);

        // ROTATE TO FIRE AT MIDDLE POWERSHOT

        // FIRE 2

        // ROTATE TO FIRE AT LEFT POWERSHOT

        // FIRE 3

        // ALIGN PERPENDICULAR WITH WHITE LINE
        //turnPID(0,0.6/180,0.00005,0.1,5000);

        // MOVE FORWARD INTO WHITE LINE

        driveAdjust(-1, 53, -170, 5000);


        telemetry.addData("auto:", "complete");
        telemetry.update();
    }
}
