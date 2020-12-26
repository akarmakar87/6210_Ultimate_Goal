package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpPractice", group = "teleop")
//@Disabled
public class TeleOpPractice extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        final double STICK_THRESHHOLD = 0.05;
        final double STICK_SCALE = 7;
        final double POWER_CAP = 1.0;

        //sets up imu and inits all motors
        init(hardwareMap, false);

        telemetry.addData("teleop:", "waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Movement

            double inputXlf = 0, inputYlf = 0, inputXrf = 0, inputYrf = 0;
            double inputXlb = 0, inputYlb = 0, inputXrb = 0, inputYrb = 0;
            double inputTurn = 0;

            if (Math.abs(gamepad1.left_stick_x) > STICK_THRESHHOLD) {
                inputXlf = gamepad1.left_stick_x * STICK_SCALE;
            }
            if (Math.abs(gamepad1.left_stick_y) > STICK_THRESHHOLD) {
                inputYlf = gamepad1.left_stick_y * STICK_SCALE;
            }
            if (Math.abs(gamepad1.left_stick_x) > STICK_THRESHHOLD) {
                inputXrf = gamepad1.left_stick_x * STICK_SCALE;
            }
            if (Math.abs(gamepad1.left_stick_y) > STICK_THRESHHOLD) {
                inputYrf = gamepad1.left_stick_y * -STICK_SCALE;
            }
            if (Math.abs(gamepad1.left_stick_x) > STICK_THRESHHOLD) {
                inputXlb = gamepad1.left_stick_x * STICK_SCALE;
            }
            if (Math.abs(gamepad1.left_stick_y) > STICK_THRESHHOLD) {
                inputYlb = gamepad1.left_stick_y * STICK_SCALE;
            }
            if (Math.abs(gamepad1.left_stick_x) > STICK_THRESHHOLD) {
                inputXrb = gamepad1.left_stick_x * STICK_SCALE;
            }
            if (Math.abs(gamepad1.left_stick_y) > STICK_THRESHHOLD) {
                inputYrb = gamepad1.left_stick_y * -STICK_SCALE;
            }

            // Right stick movement on x axis directs turning

            if (Math.abs(gamepad1.right_stick_x) > STICK_THRESHHOLD) {
                inputTurn = gamepad1.right_stick_x * STICK_SCALE;
            }

            double inputLF = inputXlf + inputYlf + inputTurn;
            double inputRF = inputXrf + inputYrf - inputTurn;
            double inputLB = inputXlb + inputYlb + inputTurn;
            double inputRB = inputXrb + inputYrb - inputTurn;

            LF.setPower(Range.clip(inputLF, -POWER_CAP, POWER_CAP));
            RF.setPower(Range.clip(inputRF, -POWER_CAP, POWER_CAP));
            LB.setPower(Range.clip(inputLB, -POWER_CAP, POWER_CAP));
            RB.setPower(Range.clip(inputRB, -POWER_CAP, POWER_CAP));

            // Intake

            if(gamepad1.right_bumper){
                // intakeL.setPower(-1);
                // intakeR.setPower(-1);
            }
            else if(gamepad2.left_bumper){
               // intakeL.setPower(1);
               // intakeR.setPower(1);
            }
            else{
                // intakeL.setPower(0);
                // intakeR.setPower(0);
            }

            // Output

            // Arm

        }

        telemetry.addData("teleop:", "complete");
        telemetry.update();
    }
}
