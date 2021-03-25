package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name = "Red2t", group = "auto")

public class Red2t extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() {
        init(hardwareMap, 1);
        initOpenCV();

        //  pos = detectStack();

        int pos = 1;

        int angle1 = 0;
        int dist1 = 0;


        switch (pos) {
            case 0:
                waitForStart();

                strafeAdjust(1, 8, 0, 3000);
                driveAdjust(0.8, 50, 0, 5000);

                shooter.setPower(.85);

                setWobbleArm(true);
                setWobbleClaw(false);

                sleep(500);

                driveAdjust(-1, 5, 0, 5000);

                turnPID(178, 0.6 / 180, 0.00005, 0.1, 3000);


                angle1 = 165;
                dist1 = 29;
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

                turnPID(180, 0.6 / 180, 0.00005, 0.1, 3000);


                angle1 = 190;
                dist1 = 50;
                break;

            case 4:
                waitForStart();

                strafeAdjust(1, 8, 0, 3000);
                driveAdjust(1, 100, 0, 5000);

                shooter.setPower(.85);

                setWobbleArm(true);
                setWobbleClaw(false);

                sleep(1000);

                driveAdjust(-1, 50, 0, 5000);

                turnPID(90, 0.6 / 180, 0.00005, 0.1, 3000);
                turnPID(181, 0.6 / 180, 0.00005, 0.1, 3000);

                angle1 = 176;
                dist1 = 76;
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

        turnPID(130, 0.6 / 180, 0.00005, 0.1, 3000);

        driveAdjust(1, 27, 0, 5000);

        setWobbleClaw(true);
        sleep(1000);
        setWobbleArm(false);
        sleep(500);

        turnPID(angle1, 0.6 / 180, 0.00005, 0.1, 3000);

        driveAdjust(-1, dist1, 0, 5000);

        turnPID(125, 0.6 / 180, 0.00005, 0.1, 3000);
        turnPID(-90, 0.6 / 180, 0.00005, 0.1, 3000);

        setWobbleArm(true);

        setWobbleClaw(false);

        setWobbleArm(false);

        if (pos == 4) {
            turnPID(180, 0.6 / 180, 0.00005, 0.1, 3000);
            driveAdjust(1, 30, 0, 5000);
        } else if (pos == 1) {
            turnPID(180, 0.6 / 180, 0.00005, 0.1, 3000);
            driveAdjust(1, 15, 0, 5000);
        }
    }
}
