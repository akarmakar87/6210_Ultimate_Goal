package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SkystoneLinearOpMode;
import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name="Red 2a", group = "auto") // RED SIDE

public class Red2a extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //sets up imu and vuforia
        init(hardwareMap, true);
        initBitmapVuforia();

        int pos = 0;
        double adjust = 0.0;
        double longAdjust = 0.0;

        //CHOSE WHETHER SHOOTING FOR HIGH GOAL OR POWERSHOT (BUTTON ON GAMEPAD)

        waitForStart();

        // GRAB WOBBLE
        //deploy arm
        //clamp wobble
        //retract arm

        // DETECT # OF RINGS
        driveAdjust(0.6, 5, 0, 2000);
        pos = detectStack(getBitmap(), true);
        pos=1;

        // ALIGN WITH DEPOT
        driveAdjust(0.8, 36 + 23*pos, 0, 5000);

        // TURN TOWARD DEPOT
        turnPID(-90,0.8/180,0.0001,0.5,5000);
        if (pos == 1 || pos == 3)
            driveAdjust(0.6, 14, -90, 3);

        // RELEASE WOBBLE
        sleep(2000);
        //deploy arm
        //release wobble
        //retract arm
        if (pos == 1 || pos == 3)
            driveAdjust(-0.6, 14, -90, 3);

        // ALIGN PERPENDICULAR TO WHITE LINE
        turnPID(180,0.6/180,0.0001,0.01,5000);

        // BACK UP PAST WHITE LINE
        driveAdjust(0.8, 36 + 24*pos, 180, 5000);

        // ROTATE TO FIRE AT RIGHT POWERSHOT
        //turnPID(180,0.8/180,0.0001,0.01,5000);

        // FIRE 1
        startShooter(1);
        sleep(2000);
        setLoader(true);
        setLoader(false);
        setLoader(true);
        setLoader(false);
        setLoader(true);
        setLoader(false);
        shooter.setPower(0);

        // ROTATE TO FIRE AT MIDDLE POWERSHOT

        // FIRE 2

        // ROTATE TO FIRE AT LEFT POWERSHOT

        // FIRE 3

        // ALIGN PERPENDICULAR WITH WHITE LINE
        turnPID(0,0.6/180,0.00005,0.1,5000);

        // MOVE FORWARD INTO WHITE LINE
        driveAdjust(0.8, 60, 0, 5000);

        telemetry.addData("auto:", "complete");
        telemetry.update();
    }
}
