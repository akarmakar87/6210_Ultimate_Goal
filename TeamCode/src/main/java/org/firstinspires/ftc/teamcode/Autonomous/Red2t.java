package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name = "Red2t", group = "auto")

public class Red2t extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() {
        init(hardwareMap, 1);
        initOpenCV();

        int pos = detectStack();

        int angle1 = 0;
        int dist1 = 0;


        switch (pos) {
            case 0:
                waitForStart();

                strafeAdjust(1, 10, 0, 3000);
                driveAdjust(0.8, 60, 0, 5000);

                shooter.setPower(.85);

                setWobbleArm(true);
                setWobbleClaw(false);

                sleep(500);

                driveAdjust(-1, 11, 0, 5000);

                turnPID(178, 0.6 / 180, 0.00005, 0.1, 3000);


                angle1 = 168;
                dist1 = 31;
                break;

            case 1:
                waitForStart();

                driveAdjust(1, 80, 0, 5000);

                setWobbleArm(true);
                setWobbleClaw(false);

                sleep(1000);

                shooter.setPower(.85);

                driveAdjust(-1, 30, 0, 5000);

                strafeAdjust(1, 8, 0, 3000);

                turnPID(90, 0.6 / 180, 0.00005, 0.1, 3000);
                turnPID(181, 0.6 / 180, 0.00005, 0.1, 3000);


                angle1 = 190;
                dist1 = 50;
                break;

            case 4:
                waitForStart();

                strafeAdjust(1, 10, 0, 3000);
                driveAdjust(1, 102, 0, 5000);

                shooter.setPower(.85);

                setWobbleArm(true);
                setWobbleClaw(false);

                sleep(1000);

                driveAdjust(-1, 50, 0, 5000);

                turnPID(90, 0.6 / 180, 0.00005, 0.1, 3000);
                turnPID(181, 0.6 / 180, 0.00005, 0.1, 3000);

                angle1 = 173;
                dist1 = 78;
                break;

        }

        setLoader(true);
        setLoader(false);

        sleep(400);

        setLoader(true);
        setLoader(false);

        sleep(400);

        setLoader(true);
        setLoader(false);

        shooter.setPower(0);

        turnPID(132, 0.6 / 180, 0.00005, 0.1, 3000);

        driveAdjust(1, 29, 0, 5000);

        setWobbleClaw(true);
        sleep(1000);
        setWobbleArm(false);
        sleep(500);

        turnPID(angle1, 0.6 / 180, 0.00005, 0.1, 3000);

        driveAdjust(-1, dist1, 0, 5000);

        if (pos == 0) {
            turnPID(110, 0.6 / 180, 0.00005, 0.1, 3000);
            turnPID(-90, 0.6 / 180, 0.00005, 0.1, 3000);
        }

        setWobbleArm(true);

        setWobbleClaw(false);

        setWobbleArm(false);

        if (pos == 4) {
            turnPID(180, 0.6 / 180, 0.00005, 0.1, 3000);
            driveAdjust(1, 30, 0, 5000);
        } else if (pos == 1) {
            turnPID(180, 0.6 / 180, 0.00005, 0.1, 3000);
            driveAdjust(1, 15, 0, 5000);
        } else if (pos == 0) {
            driveAdjust(-1, 5, 0, 5000);
            turnPID(20, 0.6 / 180, 0.00005, 0.1, 3000);
            driveAdjust(1, 10, 0, 5000);
        }
    }
}
