package org.firstinspires.ftc.teamcode.Autonomous.OldAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name="Red 2a", group = "auto") // RED SIDE
@Disabled
public class Red2 extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //sets up imu and vuforia
        init(hardwareMap, -1);
        initOpenCV();

        int pos, dist = 0;
        int ag1 = 180;
        int ag2 = 180;
        int ag3 = 180;
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
        switch (pos) {
            case 0:
                dist = 60;
                wait = 1250;
                ag1 = -175;
                ag2 = -179;
                ag3 = -177;
                break;
            case 1:
                dist = 75;
                wait = 750;
                ag1 = 179;
                ag2 = 175;
                ag3 = 171;
                break;
            case 4:
                dist = 110;
                wait = 500;
                ag1 = -175;
                ag2 = -179;
                ag3 = -177;
                break;
        }

        driveAdjust(1, dist, 0, 5000);

        // TURN TOWARD DEPOT

        if (pos == 0 || pos == 4) {
            turnPID(-90, 0.8 / 180, 0.0001, 0.5, 5000);
            driveAdjust(0.8, 22, -90, 3);
        }
        else
            turnPID(-45, 0.8 / 180, 0.0001, 0.5, 5000);

        // RELEASE WOBBLE
        //deploy arm
        setWobbleArm(true);
        //release wobble
        setWobbleClaw(false);
        //retract arm
        setWobbleArm(false);

        //if (pos == 1 || pos == 3)
        //    driveAdjust(-0.8, 14, -90, 3);

        // ALIGN PERPENDICULAR TO WHITE LINE
        turnPID(180,0.8/180,0.0001,0.01,5000);

        // BACK UP PAST WHITE LINE
        driveAdjustShooter(0.8, dist - 27, 180, 5000, 1);

        // ROTATE TO FIRE AT RIGHT POWERSHOT
        //turnPID(180,0.8/180,0.0001,0.01,5000);

        // FIRE 1
        //startShooter(1);
        sleep(wait);
        turnPID(ag1,0.6/180,0.0001,0.01,1000);
        setLoader(true);
        setLoader(false);
        turnPID(ag2,0.6/180,0.0001,0.01,1000);
        setLoader(true);
        setLoader(false);
        turnPID(ag3, 0.6/180,0.0001,0.01,1000);
        setLoader(true);
        setLoader(false);
        shooter.setPower(0);

        // ALIGN PERPENDICULAR WITH WHITE LINE
        turnPID(180,0.6/180,0.00005,0.1,1000);

        // MOVE FORWARD INTO WHITE LINE
        driveAdjust(-0.8, 35, 180, 5000);

        telemetry.addData("auto:", "complete");
        telemetry.update();
    }
}
