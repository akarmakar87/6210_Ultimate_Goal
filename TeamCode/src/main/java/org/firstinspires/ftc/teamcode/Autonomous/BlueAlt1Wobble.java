package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@Autonomous(name="Blue Alt 1 Wobble", group = "auto") // RED SIDE
//@Disabled
public class BlueAlt1Wobble extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //sets up imu and vuforia
        init(hardwareMap, 1);
        initOpenCV();

        int dist = 0;
        int wait = 0;
        double adjust = 0.0;
        double longAdjust = 0.0;

        //CHOSE WHETHER SHOOTING FOR HIGH GOAL OR POWERSHOT (BUTTON ON GAMEPAD) : NOT INCLUDED

        waitForStart();

        int pos = detectStack();
        //pos = 1;
        int angle1 = 0;
        int dist1 = 0;

        //RETRACT WOBBLE ARM
        //setWobbleArm(false);

        //STRAFE RIGHT

        strafeAdjust(1, 15, 0, 3000);
        shooter.setPower(.60);

        //MOVE FORWARD
        driveAdjust(1, 50, 0, 5000);

        //TURN TO FIRE AT HIGH GOAL
        //turnPID(90, 0.6 / 180, 0.00005, 0.1, 3000);
        //turnPID(-174, 0.6 / 180, 0.00005, 0.1, 3000);

        turnPID(170, 0.6 / 180, 0.00005, 0.1, 5000);

        //FIRE 3 RINGS INTO HIGH GOAL
        setLoader(true);
        setLoader(false);

        turnPID(168, 0.4 / 180, 0.00005, 0.1, 3000);

        setLoader(true);
        setLoader(false);

        turnPID(162, 0.6 / 180, 0.00005, 0.1, 3000);

        setLoader(true);
        setLoader(false);

        shooter.setPower(0);

        //DRIVE TO BACK OF FIELD
        turnPID(0, 0.6 / 180, 0.00005, 0.1, 3000);
        driveAdjust(1, 40, 0, 5000);

         if (pos == 0){
                turnPID(90, 0.6 / 180, 0.00005, 0.1, 3000);
                driveAdjust(1, 32, 170, 5000);
                turnPID(135, 0.6 / 180, 0.00005, 0.1, 3000);
                setWobbleArm(true);
                setWobbleClaw(false);
                sleep(1000);
                setWobbleArm(false);
                setWobbleClaw(true);
                turnPID(90, 0.6 / 180, 0.00005, 0.1, 3000);
                driveAdjust(-1, 30, 90, 5000);
                turnPID(0, 0.6 / 180, 0.00005, 0.1, 3000);
                driveAdjust(-1, 15, 0, 5000);
         }
         else if (pos == 4) {
             driveAdjust(1, 22, 0, 5000);
             turnPID(90, 0.6 / 180, 0.00005, 0.1, 3000);
             driveAdjust(1, 20, 170, 5000);
             turnPID(45, 0.6 / 180, 0.00005, 0.1, 3000);
                setWobbleArm(true);
                setWobbleClaw(false);
                sleep(1000);
                setWobbleArm(false);
                sleep(1000);
                setWobbleClaw(true);
             turnPID(90, 0.6 / 180, 0.00005, 0.1, 3000);
             driveAdjust(-1, 15, 90, 5000);
             turnPID(0, 0.6 / 180, 0.00005, 0.1, 3000);
             driveAdjust(-1, 40, 0, 5000);
         }
        else{
            turnPID(70, 0.6 / 180, 0.00005, 0.1, 3000);
            setWobbleArm(true);
            setWobbleClaw(false);
            sleep(1000);
            setWobbleArm(false);
            setWobbleClaw(true);
            turnPID(0, 0.6 / 180, 0.00005, 0.1, 3000);
            //strafeAdjust(1, 5, 0, 3000);
            //turnPID(0, 0.6 / 180, 0.00005, 0.1, 3000);
            driveAdjust(-1, 20, 0, 5000);

        }

        telemetry.addData("auto:", "complete");
        telemetry.update();
    }
}
