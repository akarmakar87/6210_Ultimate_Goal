package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SkystoneLinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group = "teleop") // BLUE SIDE

public class MainTeleOp extends SkystoneLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //sets up imu and inits all motors
        init(hardwareMap, false);

        //Declares all instance variables

        waitForStart();

        //Movement Method:

        //Intake Method:

        //Output Method:

        //Arm Method:


        telemetry.addData("auto:", "complete");
        telemetry.update();
    }
}
