package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Control Award Video", group = "teleop")
@Disabled
public class ControlAwardVideo extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap, 1);
        initOpenCV();

        waitForStart();

        // SHOW A FULL, PERFECT AUTO COMPLETING ALL OBJECTIVES
        // ^ (run existing auto)

        // SHOW ROBOT DETECTING STACK AND MOVING TO CORRESPONDING SQUARE
        // ^ (run beginning of existing auto with 3 different stack scenarios)

        // SHOW AN ACCURATE 90 DEGREE PID TURN
        sleep(2000);
        turnPID(90, 0.6 / 180, 0.00005, 0.1, 3000);
        sleep(2000);

        // SHOW driveAdjust() METHOD AND PUSH THE ROBOT OFF-CENTER TO WATCH IT CORRECT ITSELF
        sleep(2000);
        driveAdjust(1, 50, 90, 5000);
        sleep(2000);

        // SHOW strafeAdjust() METHOD AND PUSH THE ROBOT OFF-CENTER TO WATCH IT CORRECT ITSELF
        sleep(2000);
        strafeAdjust(1, 50, 90, 5000);
        sleep(2000);

        // SHOW HALF SPEED BUTTON BEING PRESSED AND THE ROBOT SLOWING DOWN
        // HOLONOMIC DRIVE: SHOW JOYSTICK MOVEMENT COMPARED TO ROBOT MOVEMENT
        // FIELD-ORIENTED CONTROL: SHOW JOYSTICK MOVEMENT COMPARED TO ROBOT MOVEMENT
        // SHOW AUTO ARM IN TELEOP AND JOYSTICK FUNCTIONALITY
        // ^ (run MainTeleop to complete these tasks)

        telemetry.addData("Program:", "complete");
        telemetry.update();
    }
}
