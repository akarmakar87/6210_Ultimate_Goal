package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SkystoneLinearOpMode;
import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name="Red 1a", group = "auto") // RED SIDE

public class Red1a extends UltimateGoalLinearOpMode {

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
/*
        // GRAB WOBBLE
        //deploy arm
        //clamp wobble
        //retract arm

        // DETECT # OF RINGS
        driveAdjust(0.5, 5, 0, 2000);
        pos = detectStack(getBitmap(), false);

        // ALIGN WITH DEPOT
        driveAdjust(0.8, 24 + 24*pos, 0, 5000);

        // TURN TOWARD DEPOT
        if (pos == 1 || pos == 3)
            turnPID(-45,0.8/180,0.0001,0.5,5000);
        else
            turnPID(45,0.8/180,0.0001,0.5,5000);

        // RELEASE WOBBLE
        sleep(2000);
        //deploy arm
        //release wobble
        //retract arm
*/
        // ALIGN PERPENDICULAR TO WHITE LINE
        turnPID(180,0.8/180,0.0001,0.5,5000);
/*
        // BACK UP PAST WHITE LINE
        driveAdjust(0.8, 24*(pos-1), 180, 5000);

        // ROTATE OUTPUT TOWARD POWERSHOTS PARALLEL TO WHITE LINE
        turnPID(90,0.8/180,0.0001,0.5,5000);

        // MOVE UNTIL COLINEAR WITH MIDDLE POWERSHOT
        driveAdjust(0.8, -36, 90, 5000);

        // ROTATE OUTPUT TOWARD POWERSHOTS (OR HIGH GOAL) PERPENDICULAR TO WHITE LINE
        turnPID(180,0.8/180,0.0001,0.5,5000);
*/
        // FIRE!!!

        // ROTATE 7.125 DEGREES TO THE RIGHT (TOWARD RIGHT POWERSHOT)

        // FIRE!!!

        // ROTATE 14.25 DEGREES TO THE LEFT (TOWARD LEFT POWERSHOT)

        // FIRE!!!

        // MOVE FORWARD INTO WHITE LINE (BECOME PERPENDICULAR AGAIN BEFORE IF YOU WANT)


        telemetry.addData("auto:", "complete");
        telemetry.update();
    }
}
