package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name = "Red2t", group = "auto")

public class Red2t extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() {
        init(hardwareMap, 1);

        initOpenCV();

        waitForStart ();

        int pos = detectStack();
        pos = 4;
        int angle1 = 0;
        int dist1 = 0;


        switch (pos) {
            case 0:

                strafeAdjust(1, 12, 0, 3000);
                driveAdjust(0.8, 67, -2, 5000);

                shooter.setPower(1);

                setWobbleArm(true);
                setWobbleClaw(false);

                sleep(500);

                driveAdjust(-1, 12 , 0, 5000);

                turnPID(90, 0.6 / 180, 0.00005, 0.1, 3000);
                turnPID(180, 0.6 / 180, 0.00005, 0.1, 3000);


                angle1 = 168;
                dist1 = 31;
                break;

            case 1:

                driveAdjust(1, 85, 0, 5000);

                setWobbleArm(true);
                setWobbleClaw(false);

                sleep(1000);

                shooter.setPower(1);

                driveAdjust(-1, 32, 0, 5000);

                strafeAdjust(1, 12, 0, 3000);

                turnPID(90, 0.6 / 180, 0.00005, 0.1, 3000);
                turnPID(179, 0.6 / 180, 0.00005, 0.1, 3000);


                angle1 = 190;
                dist1 = 50;
                break;

            case 4:

                strafeAdjust(1, 12, 0, 3000);
                driveAdjust(1, 108, -2,  5000);

                shooter.setPower(1);

                setWobbleArm(true);
                setWobbleClaw(false);

                sleep(400);

                driveAdjust(-1, 57, 0, 5000);

                turnPID(90, 0.6 / 180, 0.00005, 0.1, 3000);
                turnPID(179, 0.6 / 180, 0.00005, 0.1, 3000);

                angle1 = 165;
                dist1 = 78;
                break;

        }

        setLoader(true);
        setLoader(false);

        sleep(600);

        setLoader(true);
        setLoader(false);

        sleep(600);

        setLoader(true);
        setLoader(false);

        shooter.setPower(0);

        if (pos == 0) {
            turnPID(129, 0.6 / 180, 0.00005, 0.1, 3000);
        } else {
            turnPID(130, 0.6 / 180, 0.00005, 0.1, 3000);
        }

        driveAdjust(1, 32, 0, 5000);

        setWobbleClaw(true);
        sleep(300);
        setWobbleArm(false);
        sleep(300);

        intake.setPower(-1);

        turnPID(angle1-180, 0.6 / 180, 0.00005, 0.1, 3000);

        driveAdjust(1, dist1, 0, 5000);

        if (pos == 0) {
            turnPID(110,   0.6 / 180, 0.00005, 0.1, 3000);
            turnPID(-90, 0.6 / 180, 0.00005, 0.1, 3000);
        } else if (pos == 4) {
            turnPID(-40, 0.6 / 180, 0.00005, 0.1, 3000);
        } else {
            turnPID(-75, 0.6 / 180, 0.00005, 0.1, 3000);
            driveAdjust(1, 2, 0, 5000);
        }

        setWobbleArm(true);

        setWobbleClaw(false);

        sleep(400);
        setWobbleArm(false);
        //wobbleArm.setTargetPosition(100);

        if (pos == 4) {
            turnPID(181, 0.6 / 180, 0.00005, 0.1, 3000);
            driveAdjust(1,  50, 0, 5000);
        } else if (pos == 1) {
            driveAdjust(-1, 5, 0, 5000);
        } else if (pos == 0) {
            driveAdjust(-1, 5, -90, 5000);
            turnPID(5, 0.6 / 180, 0.00005, 0.1, 3000);
            driveAdjust(1, 12, 0,    5000);
        }

        if (pos == 1) {
            turnPID(200, 0.6 / 180, 0.00005, 0.1, 3000);
            intake.setPower(-1);
            sleep(2000);
            driveAdjust(1, 30, 0, 3000);
            turnPID(180, 0.6 / 180, 0.00005, 0.1, 3000);

            setLoader(true);
            setLoader(false);

            driveAdjust(-1, 25, 0, 3000);

        }

    }
}
