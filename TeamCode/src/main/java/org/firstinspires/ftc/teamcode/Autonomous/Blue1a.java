package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.SkystoneLinearOpMode;

@Autonomous(name="Blue 1a", group = "auto") // BLUE SIDE
@Disabled
public class Blue1a extends SkystoneLinearOpMode { //<-- MAKE SURE TO SWITCH TO ULTIMATE GOAL LINEAR OPMODE!!!!!!!

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

        // DETECT # OF RINGS

        // STRAFE ROBOT INTO WOBBLE

        // GRAB WOBBLE

        // ROTATE SO WOBBLE FACES DIRECTION OF DEPOT
        //if(a || c) { woble on left } else {wobble on right}

        // DRIVE TOWARD DEPOT

        // RELEASE WOBBLE

        // BACK UP TO WHITE LINE + 1'

        // ROTATE OUTPUT TOWARD POWERSHOTS PARALLEL TO WHITE LINE

        // MOVE UNTIL COLINEAR WITH MIDDLE POWERSHOT

        // ROTATE OUTPUT TOWARD POWERSHOTS (OR HIGH GOAL) PERPENDICULAR TO WHITE LINE

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
