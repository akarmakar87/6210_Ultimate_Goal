package org.firstinspires.ftc.teamcode.Autonomous.OldAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name="Blue 1a", group = "auto") // BLUE SIDE
@Disabled
public class Blue1 extends UltimateGoalLinearOpMode { //<-- MAKE SURE TO SWITCH TO ULTIMATE GOAL LINEAR OPMODE!!!!!!!

    @Override
    public void runOpMode() throws InterruptedException {

        //sets up imu and vuforia
        init(hardwareMap, -1);
        initOpenCV();

        int pos, dist = 0;
        int wait = 0;
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
        pos = detectStack();

        // ALIGN WITH DEPOT
        driveAdjust(0.8, 24 + 25*pos, 0, 5000);

        // TURN TOWARD DEPOT
        switch (pos) {
            case 0:
                dist = 55+5;
                wait = 1250;
                break;
            case 1:
                dist = 72+5;
                wait = 750;
                break;
            case 4:
                dist = 100+5;
                wait = 500;
                break;
        }

        driveAdjust(0.8, dist, 0, 5000);

        // RELEASE WOBBLE
        //deploy arm
        setWobbleArm(true);
        //release wobble
        setWobbleClaw(false);
        //retract arm
        setWobbleArm(false);

        // ALIGN PERPENDICULAR TO WHITE LINE
        turnPID(180,0.6/180,0.00005,0.1,5000);

        // BACK UP PAST WHITE LINE
        driveAdjust(0.8, dist-12, 180, 5000);

        // ROTATE TO FIRE AT RIGHT POWERSHOT
        // turnPID(180,0.6/180,0.00005,0.1,5000);

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
        driveAdjust(0.8, 48, 0, 5000);


        telemetry.addData("auto:", "complete");
        telemetry.update();
    }
}
