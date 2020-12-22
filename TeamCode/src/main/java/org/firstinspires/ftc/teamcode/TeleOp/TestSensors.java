package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.UltimateGoalLinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TestSensors", group = "teleop")
//@Disabled
public class TestSensors extends UltimateGoalLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        init(hardwareMap, true);

        telemetry.addData("teleop:", "waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (Math.abs(gamepad1.left_stick_x) > 0.05){
                setMotorPowers("STRAFE", gamepad1.left_stick_x,0,0,0);
            }else{
                setMotorPowers("ALL",0,0,0,0);
            }
            telemetry.addData("IMU:", get180Yaw());
            motorTelemetry();
            telemetry.update();

        }

        telemetry.addData("teleop:", "complete");
        telemetry.update();
    }
}
