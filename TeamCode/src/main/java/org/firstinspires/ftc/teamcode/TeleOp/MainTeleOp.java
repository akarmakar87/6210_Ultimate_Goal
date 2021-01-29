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
        boolean incrementTurn = false;
        boolean field = false;
        boolean manuel = true;

        int lowestArm = 0;
        double turnAngle = 0;
        final int IN_VALUE = 30;
        final int OUT_VALUE = 125;
        final int INCREMENT = 8;

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

            /**
             * GAMEPAD 1
             *
             * Controls:
             * Right Joystick - Robot Movement (Arcade Drive)
             * Left Joystick - Robot Orientation
             * Right Bumper - Intake in , Left Bumper - Intake out
             * X - Half Speed
             * Y - Switch Drive Modes (Robot Oriented or Field Oriented)
             * A - Reset Gyro
             * DPAD: LEFT / RIGHT - increment left / increment right
             * DPAD: UP / DOWN - toward goal / back field
             *
             */

            // Reset angle (0 to 360 angle)
            if (isPressed("1a", gamepad1.a)){
                zeroAng = get180Yaw();
                if (zeroAng < 0) zeroAng += 360;
            }

            // Half time toggle
            if (isPressed("1x", gamepad1.x)){
                halfspeed = !halfspeed;
            }

            // Drive mode toggle
            if (isPressed("1y", gamepad1.y)){
                field = !field;
            }

            // Turn increments
            if (isPressed("1dl", gamepad1.dpad_left)){
                incrementTurn = true;
                turnAngle = get180Yaw() + INCREMENT;
            }

            if (isPressed("1dr", gamepad1.dpad_right)){
                incrementTurn = true;
                turnAngle = get180Yaw() - INCREMENT;
            }

            if (incrementTurn && Math.abs(get180Yaw() - turnAngle) <= 1){
                incrementTurn = false;
            }

            // Movement
            double yAxis, xAxis, zAxis;

            //CONTROLLER INPUTS
            yAxis = -gamepad1.left_stick_y;
            xAxis = gamepad1.left_stick_x;
            zAxis = gamepad1.right_stick_x * 0.75;

            //MOTOR POWERs

            if (Math.abs(gamepad1.left_stick_x) > STICK_THRESHHOLD ||
                    Math.abs(gamepad1.left_stick_y) > STICK_THRESHHOLD ||
                    Math.abs(gamepad1.right_stick_x) > STICK_THRESHHOLD) {

                if (!field) motorP = holonomicDrive(yAxis, xAxis, zAxis);
                else motorP = fieldOriented(yAxis, xAxis, zAxis, zeroAng);

            }

            //TOWARDS GOAL
            else if(gamepad1.dpad_up){
                motorP = autoTurn(zeroAng, 0, 0.025);
            }
            //TOWARDS BACK
            else if (gamepad1.dpad_down){
                motorP = autoTurn(zeroAng, 180, 0.025);
            }

            else if (incrementTurn){
                motorP = autoTurn(0, turnAngle, 0.03);
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

            /**
             * GAMEPAD 2
             *
             * Controls:
             * Left Trigger - Shooter in ,  Right Trigger - Shooter out
             * B - Auto arm deploy , X - Wobble claw in/out , A - Loader in/out
             *
             */


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
                manuel = false;
                if (wobbleArm.getCurrentPosition() >= lowestArm + 75)  wobbleArm.setTargetPosition(lowestArm + IN_VALUE);
                else wobbleArm.setTargetPosition(lowestArm + OUT_VALUE);
                armTime = time.milliseconds();
            }

            if (deployArm) {
                wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleArm.setPower(1);
            }else if (!manuel){
                wobbleArm.setPower(0);
            }

            if ( (wobbleArm.getCurrentPosition() >= lowestArm + OUT_VALUE || wobbleArm.getCurrentPosition() <= lowestArm + IN_VALUE) && armTime + 1500 <= time.milliseconds()){
                deployArm = false;
            }

            if (wobbleArm.getCurrentPosition() < lowestArm) lowestArm = wobbleArm.getCurrentPosition();

            if(Math.abs(gamepad2.right_stick_y) > 0.05 ){
                deployArm = false;
                manuel = true;
                wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                wobbleArm.setPower(gamepad2.right_stick_y*-0.75);
            }
            else
                wobbleArm.setPower(0);

            if (prevTime + 1000 <= time.milliseconds()){
                prevTime = time.milliseconds();
                speed = shooter.getCurrentPosition() - lastEncoder;
                lastEncoder = shooter.getCurrentPosition();
            }

            telemetry.addData("Halfspeed (X) : ", halfspeed);
            telemetry.addData("Drive Mode (Y) : ", field ? "Field Oriented" : "Robot Oriented");
            telemetry.addData("Loader (A) : ", loaderBool ? "IN" : "OUT");
            telemetry.addData("Wobble Claw (X) : ", wobbleBool ? "IN" : "OUT" );
            telemetry.addData("Shooter Speed (encoders per second) : ", speed);
            telemetry.addData("Angle : ", get180Yaw());
            telemetry.addData("Arm angle: ", wobbleArm.getCurrentPosition());
            telemetry.update();

        }
    }
}
