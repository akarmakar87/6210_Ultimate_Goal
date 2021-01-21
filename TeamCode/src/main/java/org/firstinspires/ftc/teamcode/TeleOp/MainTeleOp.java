package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MainTeleOp", group = "teleop")

public class MainTeleOp extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        final double STICK_THRESHHOLD = 0.01;
        double shooterPower = 0;
        double[] motorP = {0.0, 0.0, 0.0, 0.0};
        double zeroAng = 0;
        boolean halfspeed = false;
        boolean deployArm = false;
        boolean wobbleBool = true;
        boolean loaderBool = true;
        int lowestArm = 0;
        final int IN_VALUE = 30;
        final int OUT_VALUE = 125;

        ElapsedTime time = new ElapsedTime();
        time.reset();

        double prevTime = -1000;
        double armTime = -1000;
        int speed = 0;

        //sets up imu and inits all motors
        init(hardwareMap, 1);

        int lastEncoder = shooter.getCurrentPosition();


        telemetry.addData("teleop:", "waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Reset angle (0 to 360 angle)
            if (isPressed("1a", gamepad1.a)){
                zeroAng = get180Yaw();
                if (zeroAng < 0) zeroAng += 360;
            }

            // Half time toggle
            if (isPressed("1x", gamepad1.x)){
                halfspeed = !halfspeed;
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

            //TOWARDS GOAL
            else if(gamepad1.dpad_up){
                motorP = autoTurn(zeroAng, 0);
            }
            //TOWARDS BACK
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
            setEachPower(motorP, halfspeed);

            // Intake
            if(gamepad1.right_bumper){
                intake.setPower(-0.83);
            }
            else if(gamepad1.left_bumper){
                intake.setPower(0.83);
            }
            else{
                intake.setPower(0);
            }

            // Output

            if(gamepad2.right_trigger > 0.05){
                shooter.setPower(gamepad2.right_trigger);
            }
            else if(gamepad2.left_trigger > 0.05){
                shooter.setPower(-gamepad2.left_trigger);
            }
            else{
                shooter.setPower(0);
            }

            //Loader toggle
            if(isPressed("2a", gamepad2.a)) loaderBool = !loaderBool;

            //Depoloyed
            if (loaderBool) loader.setPosition(1);
            //Retracted
            else loader.setPosition(0);

            //Wobble claw toggle
            if(isPressed("2x", gamepad2.x)) wobbleBool = !wobbleBool;

            //Opened
            setWobbleClaw(wobbleBool);

            // Arm
            if (isPressed("2b", gamepad2.b)) {
                deployArm = true;
                if (wobbleArm.getCurrentPosition() >= lowestArm + 75)  wobbleArm.setTargetPosition(lowestArm + IN_VALUE);
                else wobbleArm.setTargetPosition(lowestArm + OUT_VALUE);
                armTime = time.milliseconds();
            }

            if (deployArm) {
                wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleArm.setPower(1);
            }else{
                wobbleArm.setPower(0);
            }

            if ( (wobbleArm.getCurrentPosition() >= lowestArm + OUT_VALUE || wobbleArm.getCurrentPosition() <= lowestArm + IN_VALUE) && armTime + 1500 <= time.milliseconds()){
                deployArm = false;
            }

            if (wobbleArm.getCurrentPosition() < lowestArm) lowestArm = wobbleArm.getCurrentPosition();

            /*if(Math.abs(gamepad2.right_stick_y) > 0.05)
                wobbleArm.setPower(gamepad2.right_stick_y*-0.75);
            else
                wobbleArm.setPower(0);*/

            if (prevTime + 1000 <= time.milliseconds()){
                prevTime = time.milliseconds();
                speed = shooter.getCurrentPosition() - lastEncoder;
                lastEncoder = shooter.getCurrentPosition();
            }

            telemetry.addData("encoders per second", speed);
            telemetry.addData("wobbleArm", wobbleArm.getCurrentPosition());
            telemetry.addData("angle", get180Yaw());
            telemetry.addData("Halfspeed (X)", halfspeed);
            telemetry.addData("arm", deployArm);
            telemetry.update();

        }
    }
}
