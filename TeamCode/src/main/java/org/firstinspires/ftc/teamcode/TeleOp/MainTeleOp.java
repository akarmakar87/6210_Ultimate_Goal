package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SkystoneLinearOpMode;
import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MainTeleOp", group = "teleop")

public class MainTeleOp extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        final double STICK_THRESHHOLD = 0.05;
        final double STICK_SCALE = 7;
        final double POWER_CAP = 1.0;
        double shooterPower = 0;

        ElapsedTime time = new ElapsedTime();
        time.reset();

        double prevTime = -1000;
        int speed = 0;

        //sets up imu and inits all motors
        init(hardwareMap, false);

        int lastEncoder = shooter.getCurrentPosition();


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
                intake.setPower(0.85);
            }
            else if(gamepad1.left_bumper){
                intake.setPower(-0.85);
            }
            else{
                intake.setPower(0);
            }

            // Output

            if(gamepad1.right_trigger > 0.05){
                shooter.setPower(gamepad1.right_trigger);
            }
            else if(gamepad1.left_trigger > 0.05){
                shooter.setPower(-gamepad1.left_trigger);
            }
            else{
                shooter.setPower(0);
            }

            if(gamepad1.dpad_up){
                shooterPower = Range.clip(shooterPower+.05, -1,1);
                shooter.setPower(shooterPower);
            }
            if(gamepad1.dpad_down){
                shooterPower = Range.clip(shooterPower-.05, -1,1);
                shooter.setPower(shooterPower);
            }

            //Loader

            //Deployed
            if(gamepad1.a){
                loader.setPosition(0);
            }
            //Retracted
            if(gamepad1.b){
                loader.setPosition(1);
            }

            // Arm

            if (prevTime + 1000 <= time.milliseconds()){
                prevTime = time.milliseconds();
                speed = shooter.getCurrentPosition() - lastEncoder;
                lastEncoder = shooter.getCurrentPosition();
            }

            telemetry.addData("shooterPower:", shooterPower);
            telemetry.addData("shooter encoders:", shooter.getCurrentPosition());
            telemetry.addData("time:", time.milliseconds());
            telemetry.addData("encoders per second", speed);
            telemetry.update();

        }
    }
}
