package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name="Red Alt", group = "auto") // RED SIDE
//@Disabled
public class RedAlt extends UltimateGoalLinearOpMode {

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

        //RETRACT WOBBLE ARM
        //setWobbleArm(false);

        //STRAFE LEFT
        strafeAdjust(-1, 12, 0, 3000);
        shooter.setPower(.78);

        //MOVE FORWARD
        driveAdjust(1, 50, 0, 5000);

        //TURN TO FIRE AT HIGH GOAL
        turnPID(90, 0.6 / 180, 0.00005, 0.1, 3000);
        turnPID(145, 0.6 / 180, 0.00005, 0.1, 3000);

        //FIRE 3 RINGS INTO HIGH GOAL
        setLoader(true);
        setLoader(false);


        setLoader(true);
        setLoader(false);


        setLoader(true);
        setLoader(false);

        shooter.setPower(0);

        //DRIVE TO PARKING SPOT (BACKWARDS)
        turnPID(0, 0.6 / 180, 0.00005, 0.1, 3000);
        driveAdjust(1, 20, 0, 5000);

        //EVIL CODE (COMMENT OUT IF YOU WANT TO BE NICE)
        //turnPID(90, 0.6 / 180, 0.00005, 0.1, 3000);
        //driveAdjust(1, 50, 90, 5000);

        telemetry.addData("auto:", "complete");
        telemetry.update();
    }
}
