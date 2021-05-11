package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name = "BlueBagel", group = "auto")
//@Disabled
public class BlueBagel extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() {

            init(hardwareMap, -1);

            initOpenCV();

            waitForStart();

            int pos = detectStack();
            pos = 4;
            int angle1 = 0;
            int dist1 = 0;


            switch (pos) {
                case 0:

                    strafeAdjust(-1, 14, 0, 2000);
                    driveAdjust(0.8, 62, -2, 5000); //66

                    shooter.setPower(0.64);    //.6

                    setWobbleArm(true);
                    setWobbleClaw(false);

                    sleep(500);

                    driveAdjust(-1, 12, 0, 5000);   //16

                    // use new pid method to combine into single turn command with chosen direction
                    turnPID(-90, 0.6 / 180, 0.00005, 0.1, 3000);
                    turnPID(152, 0.6 / 180, 0.00005, 0.1, 3000);


                    angle1 = 180;
                    dist1 = 55;
                    break;

                case 1:
                    strafeAdjust(-1, 2, 0, 3000);
                    driveAdjust(1, 85, 0, 5000);    //87

                    turnPID(-60, 0.6 / 180, 0.00005, 0.1, 3000);

                    setWobbleArm(true);
                    setWobbleClaw(false);
                    wobbleArm.setTargetPosition(250);

                    sleep(1000);

                    turnPID(0, 0.6 / 180, 0.00005, 0.1, 3000);

                    shooter.setPower(.62);  //.68

                    driveAdjust(-1, 31, 0, 5000);   //35

                    strafeAdjust(-1, 11, 0, 3000);

                    // use new pid method to combine into single turn command with chosen direction
                    turnPID(-90, 0.6 / 180, 0.00005, 0.1, 3000);
                    turnPID(152, 0.6 / 180, 0.00005, 0.1, 3000);


                    angle1 = 0;
                    dist1 = 51;
                    break;

                case 4:

                    strafeAdjust(-1, 12, 0, 3000);
                    driveAdjust(1.75, 108, 2, 5000);

                    shooter.setPower(.63);  //9

                    setWobbleArm(true);
                    setWobbleClaw(false);

                    sleep(400);

                    driveAdjust(-1, 57, -2, 5000);

                    // use new pid method to combine into single turn command with chosen direction
                    turnPID(-90, 0.6 / 180, 0.00005, 0.1, 3000);
                    turnPID(152, 0.6 / 180, 0.00005, 0.1, 3000);

                    angle1 = 170;
                    dist1 = 73;
                    break;

            }

            setLoader(true);
            setLoader(false);


            setLoader(true);
            setLoader(false);


            setLoader(true);
            setLoader(false);

            shooter.setPower(0);


            if (pos == 0) {
                turnPID(-145, 0.6 / 180, 0.00005, 0.1, 3000);
            } else if (pos == 1) {
                intake.setPower(-1);
                turnPID(-100, 0.6 / 180, 0.00005, 0.1, 3000);
                driveAdjust(1, 30, 0, 5000);
                turnPID(-185, 0.6 / 180, 0.00005, 0.1, 3000);
                intake.setPower(0);
            } else {
                intake.setPower(-0.8);
                turnPID(-120, 0.6 / 180, 0.00005, 0.1, 3000);
                driveAdjust(1, 30, 0, 5000);
                turnPID(165, 0.6 / 180, 0.00005, 0.1, 3000);
                intake.setPower(0);
            }


            if (pos == 1) {
                setWobbleArm(true);
                driveAdjust(1, 12, 0, 5000);
            } else if (pos == 0) {
                driveAdjust(1, 32, 0, 5000);
            } else {
                driveAdjust(1, 10, 0, 5000);
            }

            setWobbleClaw(true);
            sleep(300);
            setWobbleArm(false);
            sleep(300);

            turnPID(-angle1, 0.6 / 180, 0.00005, 0.1, 3000);

            if (pos == 0) {
                driveAdjust(-1, dist1, 0, 5000);
            } else if (pos == 4) {
                driveAdjust(-1, dist1, 0, 5000);
            } else {
                driveAdjust(1, dist1, 0, 5000);
            }

            if (pos == 0) {
                turnPID(-110, 0.6 / 180, 0.00005, 0.1, 3000);
                turnPID(90, 0.6 / 180, 0.00005, 0.1, 3000);
            } else if (pos == 4) {
                turnPID(10, 0.6 / 180, 0.00005, 0.1, 3000);
            }

            setWobbleArm(true);

            setWobbleClaw(false);

           /* intake.setPower(.3);
           // sleep(400);
          //  intake.setPower(0);
          //  setWobbleArm(false);*/
            //wobbleArm.setTargetPosition(100);

            if (pos == 4) {
                shooter.setPower(.70);
                wobbleArm.setTargetPosition(200);
                intake.setPower(1);
                driveAdjust(-1, 41, -30, 5000); //1, 35, 180
                turnPID(168, 0.6 / 180, 0.00005, 0.1, 3000);    //160
                intake.setPower(0);
                setLoader(true);
                setLoader(false);
                setLoader(true);
                setLoader(false);
                driveAdjust(-1, 7, 180, 5000);
                shooter.setPower(0);
            } else if (pos == 1) {
                shooter.setPower(.575);
                driveAdjust(-1, 16, 0, 5000);
                turnPID(130, 0.6 / 180, 0.00005, 0.1, 3000);
                setLoader(true);
                setLoader(false);
                shooter.setPower(0);
                driveAdjust(-1, 12, 180, 5000);
            } else if (pos == 0) {
                driveAdjust(-1, 5, -90, 5000);
                turnPID(0, 0.6 / 180, 0.00005, 0.1, 3000);
                //driveAdjust(1, 12, 0, 5000);
            }

        }
}





