package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MainTeleOp", group = "teleop")

public class MainTeleOp extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        final double STICK_THRESHHOLD = 0.05;
        final double STICK_SCALE = 7;
        final double POWER_CAP = 1.0;
        double shooterPower = 0;
        double[] motorP = {0.0, 0.0, 0.0, 0.0};
        double zeroAng = 0;
        boolean halfspeed = false;

        ElapsedTime time = new ElapsedTime();
        time.reset();

        double prevTime = -1000;
        int speed = 0;

        //sets up imu and inits all motors
        init(hardwareMap, 0);

        int lastEncoder = shooter.getCurrentPosition();


        telemetry.addData("teleop:", "waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Reset angle (0 to 360 angle)
            if (gamepad1.a){
                zeroAng = get180Yaw();
                if (zeroAng < 0) zeroAng += 360;
            }

            // Movement
            double yAxis, xAxis, zAxis;

            //CONTROLLER INPUTS
            yAxis = -gamepad1.left_stick_y;
            xAxis = gamepad1.left_stick_x;
            zAxis = gamepad1.right_stick_x * 0.75;

            if (Math.abs(gamepad1.left_stick_x) > STICK_THRESHHOLD ||
                    Math.abs(gamepad1.left_stick_y) > STICK_THRESHHOLD ||
                    Math.abs(gamepad1.right_stick_x) > STICK_THRESHHOLD) {

                motorP = holonomicDrive(yAxis, xAxis, zAxis);

            }

            else if (gamepad1.dpad_down){
                motorP = autoTurn(zeroAng, 180);
            }

            else {

                motorP[0] = 0;
                motorP[1] = 0;
                motorP[2] = 0;
                motorP[3] = 0;

            }

            //SET MOTOR POWERS
            LF.setPower(Range.clip(motorP[0], -POWER_CAP, POWER_CAP));
            RF.setPower(Range.clip(motorP[1], -POWER_CAP, POWER_CAP));
            LB.setPower(Range.clip(motorP[2], -POWER_CAP, POWER_CAP));
            RB.setPower(Range.clip(motorP[3], -POWER_CAP, POWER_CAP));

            // Intake

            if(gamepad1.right_bumper){
                intake.setPower(-0.85);
            }
            else if(gamepad1.left_bumper){
                intake.setPower(0.85);
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
            if(Math.abs(gamepad2.right_stick_y) > 0.05)
                wobbleArm.setPower(gamepad2.right_stick_y*-0.75);
            else
                wobbleArm.setPower(0);

            if(gamepad2.a)
                setWobbleClaw(true);
            if(gamepad2.b)
                setWobbleClaw(false);

            if (prevTime + 1000 <= time.milliseconds()){
                prevTime = time.milliseconds();
                speed = shooter.getCurrentPosition() - lastEncoder;
                lastEncoder = shooter.getCurrentPosition();
            }

            telemetry.addData("shooterPower:", shooterPower);
            telemetry.addData("shooter encoders:", shooter.getCurrentPosition());
            telemetry.addData("time:", time.milliseconds());
            telemetry.addData("encoders per second", speed);
            telemetry.addData("wobbleArm", wobbleArm.getCurrentPosition());
            telemetry.update();

        }
    }
}
