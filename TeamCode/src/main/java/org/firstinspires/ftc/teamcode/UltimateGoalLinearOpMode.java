package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class UltimateGoalLinearOpMode extends LinearOpMode {

    // DECLARE VARIABLES TO BE USED
    ElapsedTime runtime;

    //MOTORS
    public DcMotor LF;
    public DcMotor RF;
    public DcMotor LB;
    public DcMotor RB;

    //SENSORS
    public BNO055IMU imu;

    //GYRO VARIABLES
    Orientation angles;

    //ticks per inch = (Motor revolutions * gear up ratio) / (wheel diameter * pie)
    //Motor revolutions = COUNTS_PER_MOTOR_REV
    //gear up ratio = 2:1   (ratio beyond motor)
    //wheel diameter = WHEEL_DIAMETER_INCHES
    static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // REV Motor Encoder (1120 for 40:1) (560 for 20:1) (336 for 12:1)
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;   // This is < 1.0 if geared UP   (ratio is 2:1)
    static final double     WHEEL_DIAMETER_INCHES   = 2 * (3 + (15 / 16));     // For figuring circumference

    public double encoderToInches = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * Math.PI);
    //Multiply by desired distance (inches)

    // INITIALIZE
    public void init(HardwareMap map, boolean auto){

        runtime     = new ElapsedTime();
        LF          = map.dcMotor.get("LF");
        RF          = map.dcMotor.get("RF");
        LB          = map.dcMotor.get("LB");
        RB          = map.dcMotor.get("RB");
        imu         = map.get(BNO055IMU.class, "imu"); // Check which IMU is being used

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);//r
        RB.setDirection(DcMotorSimple.Direction.FORWARD);//r
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //SET UP GYRO
        angles = new Orientation();

        resetEncoders();

        if (auto) {
            //ACTIONS HERE
        }

        BNO055IMU.Parameters bparameters = new BNO055IMU.Parameters();
        bparameters.mode = BNO055IMU.SensorMode.IMU;
        bparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        bparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        bparameters.loggingEnabled = false;

        imu.initialize(bparameters);

        angles = imu.getAngularOrientation(); //GET ORIENTATION

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("IMU calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        telemetry.addData("Status: ", "All Initialized");
        telemetry.update();
    }

    //RESET ENCODERS
    public void resetEncoders(){
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();
    }

    //GET ENCODER AVERAGE
    /**NOTE:
     * Make sure all encoders work before dividing by 4 or put in a condition to determine if
     * an encoder value is weird and divide accordingly
     */
    public int getEncoderAvg(){
        int avg = (Math.abs(LF.getCurrentPosition())
                + Math.abs(LB.getCurrentPosition())
                + Math.abs(RB.getCurrentPosition())
                + Math.abs(RF.getCurrentPosition()))/4;
        return avg;
    }

    //SET MOTOR POWERS

    //STRAFE

    //DRIVE FORWARD/BACKWARD

    //PID TURNING

    //DETECT NUMBER OF RINGS

    //INTAKE RINGS

    //SHOOT RINGS (DO PHYSICS CALCULATIONS)

    //GRAB & RELEASE WOBBLE

    //******************************************************************************//

    // AUTONOMOUS METHODS:

    // ONE METHOD FOR MOVING BACK AND FORTH BETWEEN DROP ZONES AND LAUNCH LINE (PARAMS: WHICH BOX TO GO TO, WHICH LINE START AT)

    // ONE METHOD TO DETECT THE NUMBER RINGS STACKED IN AUTO (PARAMS: BITMAP OBJECT)

    // ONE METHOD TO DO POWERSHOTS IN AUTO (PARAMS: BOX)

    // ONE METHOD WITH COLOR SENSOR TO LINE UP WITH LAUNCH LINE

    // ONE METHOD TO SET ___

    //



    @Override
    public void runOpMode() throws InterruptedException {}
}