package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name = "PowerShotAuto", group = "auto")

public class PowerShotAuto extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() {

        init(hardwareMap, 1);

        initOpenCV();

        waitForStart ();

        int pos =  detectStack();
        int dropDist = 0;
        int wobbleAngle = 0;
        int wobbleDist1 = 0;
        int wobbleDist2 = 0;
        int wobbleDist3 = 0;
        int parkDist = 0;

        switch (pos) {
            case 0:

                 dropDist = 0;

                 wobbleAngle = 0;

                 wobbleDist1 = 0;

                 wobbleDist2 = 0;

                 wobbleDist3 = 0;

                 parkDist = 0;

                 break;

            case 1:

                dropDist = 0;

                wobbleAngle = 0;

                wobbleDist1 = 0;

                wobbleDist2 = 0;

                wobbleDist3 = 0;

                parkDist = 0;

                break;

            case 4:

                dropDist = 0;

                wobbleAngle = 0;

                wobbleDist1 = 0;

                wobbleDist2 = 0;

                wobbleDist3 = 0;

                parkDist = 0;

                break;

        }

        shooter.setPower(.6);

        driveAdjust(1, 0, 0, 5000);

        turnPID(0, 0.6 / 180, 0.00005, 0.1, 3000);

        setLoader(true);
        setLoader(false);

        turnPID(0, 0.6 / 180, 0.00005, 0.1, 3000);

        setLoader(true);
        setLoader(false);

        turnPID(0, 0.6 / 180, 0.00005, 0.1, 3000);

        setLoader(true);
        setLoader(false);

        turnPID(0, 0.6 / 180, 0.00005, 0.1, 3000);

        if (pos == 1) {
            strafeAdjust(1, 0, 0, 3000);
        }


        driveAdjust(1, dropDist, 0, 5000);

        turnPID(wobbleAngle, 0.6 / 180, 0.00005, 0.1, 3000);

        driveAdjust(1, wobbleDist1, 0, 5000);

        setWobbleArm(true);
        setWobbleClaw(false);

        sleep(400);

        driveAdjust(-1, wobbleDist2, 0, 5000);

        turnPID(wobbleAngle - 180, 0.6 / 180, 0.00005, 0.1, 3000);

        driveAdjust(1, wobbleDist3, 0, 5000);

        setWobbleClaw(true);
        setWobbleArm(false);

        sleep(400);

        driveAdjust(-1, wobbleDist3, 0, 5000);

        turnPID(wobbleAngle, 0.6 / 180, 0.00005, 0.1, 3000);

        driveAdjust(1, wobbleDist2-10, 0, 5000);

        setWobbleArm(true);
        setWobbleClaw(false);

        driveAdjust(-1, 4, 0, 5000);

        turnPID(0, 0.6 / 180, 0.00005, 0.1, 3000);

        driveAdjust(-1, parkDist, 0, 5000);








    }
}
