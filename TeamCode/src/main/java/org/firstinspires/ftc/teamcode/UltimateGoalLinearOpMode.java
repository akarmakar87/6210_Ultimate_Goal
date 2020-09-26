package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

//Test re download

public class UltimateGoalLinearOpMode extends LinearOpMode {

    // DECLARE VARIABLES TO BE USED
    ElapsedTime runtime;

    //MOTORS, SERVOS, AND SENSORS
    public DcMotor LF;
    public DcMotor RF;
    public DcMotor LB;
    public DcMotor RB;
    public DcMotor intakeR;
    public DcMotor intakeL;
    public BNO055IMU imu;
    public DcMotor lift;
    public DcMotor arm;
    public Servo claw;
    public Servo stickL;
    public Servo stickR;
    public Servo fangL;
    public Servo fangR;
    public ColorSensor colorSensor;
    //public DistanceSensor distanceSensor;
    public Servo foundationR;
    public Servo foundationL;

    /**COLOR SENSOR VARIABLES
    float hsvValues[] = {0f, 0f, 0f};
    float values[] = hsvValues;
    **/

    //ClAW VARIABLES
    private double clawStartPosition = 0.0;
    private double clawEndPosition = 1.0;

    //ARM MOVEMENT
    private double armSpeed = 0;

    //GYRO VARIABLES
    Orientation angles;

    //TOGGLE VARIABLES
    public ArrayList<Boolean> booleanArray = new ArrayList<>();
    public int booleanIncrementer = 0;

    //ticks per inch = (Motor revolutions * gear up ratio) / (wheel diameter * pie)
    //Motor revolutions = COUNTS_PER_MOTOR_REV
    //gear up ratio = 2:1   (ratio beyond motor)
    //wheel diameter = WHEEL_DIAMETER_INCHES
     static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // REV Motor Encoder (1120 for 40:1) (560 for 20:1) (336 for 12:1)
     static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;   // This is < 1.0 if geared UP   (ratio is 2:1)
     static final double     WHEEL_DIAMETER_INCHES   = 2 * (3 + (15 / 16));     // For figuring circumference

   /* static final double     COUNTS_PER_MOTOR_REV    = 420 ;    // REV Motor Encoder (420 for 15:1) (336 for 12:1)
    static final double     DRIVE_GEAR_REDUCTION    = ? ;   // This is < 1.0 if geared UP   (ratio is 2:1)
    static final double     WHEEL_DIAMETER_INCHES   = 100 / 25.4; // GoBilda Mecanum is 100mm in diameter
    */

    public double encoderToInches = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * Math.PI); //Multiply desired distance (inches)

    //Vuforia variables
    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public static final boolean PHONE_IS_PORTRAIT = false  ;


    public static final String VUFORIA_KEY = "AQt2xVL/////AAABmXIVKUnTcEJbqvVBjp/Sw/9SqarohYyKotzRjT/Xl1/S8KDwsFHv/zYw6rXqXTjKrnjk92GfBA4hbZaQP17d1N6BiBuXO2W/hFNoMGxiF+fWlnvtDmUM1H/MF9faMOjZcPNjnQ7X8DVwdDDha3A3aqaoegefkKxb4A5EjP8Xcb0EPJ1JA4RwhUOutLbCDJNKUq6nCi+cvPqShvlYTvXoROcOGWSIrPxMEiOHemCyuny7tJHUyEg2FTd2upiQygKAeD+LN3P3cT02aK6AJbQ0DlQccxAtoo1+b//H6/eGro2s0fjxA2dH3AaoHB7qkb2K0Vl7ReFEwX7wmqJleamNUG+OZu7K3Zm68mPudzNuhAWQ";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch        = 25.4f;
    public static final float mmTargetHeight   = (6) * mmPerInch;  // the HEIGHT of the center of the target image above the floor

    // Constant (z-axis = height) for SKYSTONE BLOCK IMAGE Target
    public static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    public static final float bridgeZ = 6.42f * mmPerInch;
    public static final float bridgeY = 23 * mmPerInch;
    public static final float bridgeX = 5.18f * mmPerInch;
    public static final float bridgeRotY = 59;                                 // Units are degrees
    public static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    public static final float halfField = 72 * mmPerInch;
    public static final float quadField  = 36 * mmPerInch;

    // Class Members
    public OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer vuforia = null;

    WebcamName LogitechC310 = null;

    public boolean targetVisible = false;
    public float phoneXRotate    = 0;
    public float phoneYRotate    = 0;
    public float phoneZRotate    = 0;

    public VuforiaLocalizer vuforiaPC = null;

    VuforiaLocalizer.CloseableFrame frame; //takes the frame at the head of the queue
    Image rgb = null;

    //Vuforia stuff moved from init in order to make global variables

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    VectorF translation;
    Orientation rotation;
    VuforiaTrackables targetsSkyStone;

    //TensorFlow stuff

    public static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    public static final String LABEL_STONE = "Stone";
    public static final String LABEL_SKYSTONE = "Skystone";

    public TFObjectDetector tfod;
    //private BlockingQueue<VuforiaLocalizer.CloseableFrame> frame;


    // INITIALIZE
    public void init(HardwareMap map, boolean auto){

        runtime     = new ElapsedTime();
        LF          = map.dcMotor.get("LF");
        RF          = map.dcMotor.get("RF");
        LB          = map.dcMotor.get("LB");
        RB          = map.dcMotor.get("RB");
        intakeR     = map.dcMotor.get("iR");
        intakeL     = map.dcMotor.get("iL");
        imu         = map.get(BNO055IMU.class, "imu"); // Check which IMU is being used
        arm         = map.dcMotor.get("arm");
        lift        = map.dcMotor.get("lift");
        claw        = map.servo.get("claw");
        foundationL = map.servo.get("fL");
        foundationR = map.servo.get("fR");
        stickL      = map.servo.get("stickL");
        stickR      = map.servo.get("stickR");
        fangL       = map.servo.get("fangL");
        fangR       = map.servo.get("fangR");
        colorSensor = map.get(ColorSensor.class, "colorSensor");
        //distanceSensor = map.get(.class, "distanceSensor");

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);//r
        RB.setDirection(DcMotorSimple.Direction.FORWARD);//r
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeR.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeL.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //arm.setPower(armSpeed);

        //SET UP GYRO
        angles = new Orientation();

        resetEncoders();

        if (auto) {
            foundationD(false);
            hook(false, false);
            fang(false, false);
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

        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        //initBitmapVuforia();
        telemetry.addData("Vuforia: ", "Initialization complete");
        telemetry.update();

        telemetry.addData("Status: ", "All Initialized");
        telemetry.update();
    }

    public void initVuforia(){
        //LogitechC310 = hardwareMap.get(WebcamName.class, "LogitechC310");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        //parameters.cameraName = LogitechC310;

        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        //LIST OF ALL VUMARKS
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));


        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        targetsSkyStone.activate();

        telemetry.addData("Tracking: ", "Enabled");
        telemetry.update();

      /*  if (tfod != null) {
            tfod.activate();
        }*/

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
    }

    public void initBitmapVuforia(){

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //LogitechC310 = hardwareMap.get(WebcamName.class, "Logitech C310");

        //localizer for webcam
        VuforiaLocalizer.Parameters paramWC = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        paramWC.vuforiaLicenseKey = VUFORIA_KEY;
        //paramWC.cameraName = LogitechC310;
        vuforiaPC = ClassFactory.getInstance().createVuforia(paramWC);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforiaPC.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time

        telemetry.addData("Vuforia:", "initialized");
        telemetry.update();
    }

    /*public void initTensorFlow(){
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }*/

    public void initTfod() {
        //CREATION OF TFOD MONITOR VIEW IN ADDITION TO VUFORIA CAMERA MONITOR VIEW IS PROBABLY CAUSING TWO VIEWS
        // HOW TO FIX? SHOULD I GET RID OF ONE? OR DOES IT NOT MATTER IF THERE ARE TWO?
       int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
               "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minimumConfidence = 0.8;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKYSTONE);
   }

    public boolean ifPressed(boolean button){
        boolean output = false;

        if(booleanArray.size() == booleanIncrementer){
            booleanArray.add(false);
        }

        boolean buttonWas = booleanArray.get(booleanIncrementer);
        if(button != buttonWas && button == true){
            output = true;
        }

        booleanArray.set(booleanIncrementer, button);

        booleanIncrementer += 1;
        return output;
    }

    public boolean ifPressed(double button){
        boolean output = false;
        boolean buttonBoolean = false;

        if(button >= 0.05){
            buttonBoolean = true;
        }

        if(booleanArray.size() == booleanIncrementer){
            booleanArray.add(false);
        }

        boolean buttonWas = booleanArray.get(booleanIncrementer);
        if(buttonBoolean != buttonWas && buttonBoolean == true){
            output = true;
        }

        booleanArray.set(booleanIncrementer, buttonBoolean);

        booleanIncrementer += 1;
        return output;
    }

    //DRIVE METHODS
    public double[] holonomicDrive(double leftX, double leftY, double rightX, double correction){
        double[] motorPower = {0.0, 0.0, 0.0, 0.0};

        motorPower[0] = leftX - leftY - rightX;
        motorPower[1] = leftX - leftY + rightX;
        motorPower[2] = -leftX - leftY - rightX;
        motorPower[3] = -leftX - leftY + rightX;

        return scalePower(motorPower[0], motorPower[1], motorPower[2], motorPower[3], correction);
    }    

    /**public double[] fieldOriented(double leftX, double leftY, double rightX, double correction, double zeroAng){
        double[] motorPower = new double[4];

        double magnitude = Math.hypot(leftX, leftY); //How fast it goes (slight push is slow etc)
        double angle = Math.atan2(leftY, leftX) - Math.toRadians(getResetableYaw(zeroAng)); //Angle the joystick is turned in
        double rotation = rightX;

        motorPower[0] = magnitude * Math.sin(angle - Math.PI / 4) - rotation; //Left front motor
        motorPower[1] = magnitude * Math.sin(angle - Math.PI / 4) + rotation; //Right front motor
        motorPower[2] = magnitude * Math.sin(angle + Math.PI / 4) - rotation; //Left back motor
        motorPower[3] = magnitude * Math.sin(angle + Math.PI / 4) + rotation; //Right back motor

        return scalePower(motorPower[0], motorPower[1], motorPower[2], motorPower[3], correction);
    }**/

    public double[] proportionalPower(double lf, double rf, double lb, double rb){
        double[] power = {lf, rf, lb, rb};
        double max = 0;
        int i = 0;
        for(i = 0; i < power.length; i++){ //find the max power to scale all the powers down by it
            if(Math.abs(power[i]) > max){
                max = Math.abs(power[i]);
            }
        }

        for(i = 0; i < power.length; i++){
            power[i] /= max;
            //power[i] = Math.floor(power[i] * 100) / 100;
        }

        return power;
    }

    public double[] autoTurn(double currHeading, boolean close){
        double[] power = new double[4];
        double coefficient = 0.05;
        double mPower, error = 0;

        if(close){
            error =  0 - currHeading;
            mPower = error * coefficient;
        }else{
            error = 180 - currHeading;
            mPower = error * coefficient;
        }

        if(mPower < 0){ //negative turn left
            power[0] = -mPower;
            power[1] = mPower;
            power[2] = -mPower;
            power[3] = mPower;

        }else{ //positive turn right
            power[0] = mPower;
            power[1] = -mPower;
            power[2] = mPower;
            power[3] = -mPower;
        }


        return power;
    }

    public double[] fieldOriented(double leftX, double leftY, double rightX, double correction, double zeroAng){
        double[] motorPower = new double[4];

        double magnitude = Math.hypot(leftX, leftY); //How fast it goes (slight push is slow etc)
        double angle = Math.atan2(leftY, leftX) - Math.toRadians(getResetableYaw(zeroAng)); //Angle the joystick is turned in
        double rotation = rightX;

        motorPower[0] = magnitude * Math.sin(angle - Math.PI / 4) - rotation; //Left front motor
        motorPower[1] = magnitude * Math.sin(angle - Math.PI / 4) + rotation; //Right front motor
        motorPower[2] = magnitude * Math.sin(angle + Math.PI / 4) - rotation; //Left back motor
        motorPower[3] = magnitude * Math.sin(angle + Math.PI / 4) + rotation; //Right back motor

        motorPower = scalePower(motorPower[0], motorPower[1], motorPower[2], motorPower[3], correction);

        if(magnitude > 0.9){
            motorPower = proportionalPower(motorPower[0], motorPower[1], motorPower[2], motorPower[3]);
        }

        return motorPower;

    }

    public double[] scalePower(double LF, double RF, double LB, double RB, double correction){ //important for if we try to turn while strafing
        double[] power = {LF, RF, LB, RB};
        double max = Math.abs(power[0]);
        int index = 0;
        while(index < power.length){ //find the max power to scale all the powers down by it
            if(Math.abs(power[index]) > max){
                max = Math.abs(power[index]);
            }
            index += 1;
        }

        for(int i = 0; i < power.length; i++){
            if(power[i] < 0)
                power[i] = -(Math.abs(power[i] - correction));
            else
                power[i] -= correction;
        }

        if(max > 1.0){
            for(int i = 0; i < power.length; i++){
                power[i] /= max;
            }
        }
        /**
        if(power[0] < 0){
            power[0] = -(Math.abs(power[0] - correction));
        }else{
            power[0] -= correction;
        }
        if(power[1] < 0){
            power[1] = -(Math.abs(power[1] + correction));
        }else{
            power[1] += correction;
        }
        if(power[2] < 0){
            power[2] = -(Math.abs(power[2] + correction));
        }else{
            power[2] += correction;
        }
        if(power[3] < 0){
            power[3] = -(Math.abs(power[3] - correction));
        }else{
            power[3] -= correction;
        }**/

        return power;
    }

    public double[] strafeField(double angle, double magnitude, double correction, double zeroAng){
        double[] motorPower = {0.0, 0.0, 0.0, 0.0};
        double angles = angle - Math.toRadians(getResetableYaw(zeroAng)); //Angle the joystick is turned in

        motorPower[0] = magnitude * Math.sin(angles - Math.PI / 4); //Left front motor
        motorPower[1] = magnitude * Math.sin(angles - Math.PI / 4); //Right front motor
        motorPower[2] = magnitude * Math.sin(angles + Math.PI / 4); //Left back motor
        motorPower[3] = magnitude * Math.sin(angles + Math.PI / 4); //Right back motor

        return scalePower(motorPower[0], motorPower[1], motorPower[2], motorPower[3], correction);
    }

    public double getCorrection(double currAngle, double tAngle){
        double leftCoefficient = 0.06, rightCoefficient = 0.02, correction = 0, hError;
        hError = tAngle - currAngle;
        if(hError > 0)
            correction = hError * leftCoefficient; //strafe bad on this side so more
        else if(hError < 0)
            correction = hError * rightCoefficient; //less aggressive on this side
        return Range.clip(correction, -0.4, 0.4);
    }

    public double[] strafeCorrection(double strafePower, double correction, boolean right){
        double[] power = new double[4];
        if (right){
            power[0] = -strafePower;
            power[1] = -strafePower;
            power[2] = strafePower;
            power[3] = strafePower;
        }else {
            power[0] = strafePower;
            power[1] = strafePower;
            power[2] = -strafePower;
            power[3] = -strafePower;
        }
        return scalePower(power[0], power[1], power[2], power[3], correction);
    }

    public double[] getCorrectionPID(double currAngle, double tAngle, double kP, double kI, double kD, double time, double pastTime, double pastError){
        double[] correction = new double[3];
        double hError, prevError, dT, prevTime, currTime;

        if(Math.abs(tAngle - currAngle) > 1){
            prevError = pastError;
            hError = tAngle - currAngle;
            prevTime = pastTime;
            currTime = time;
            dT = currTime - prevTime; //Difference in time
            correction[0] = (hError * kP) + ((hError) * dT * kI) + ((hError - prevError)/dT * kD);
            correction[1] = hError;
            correction[2] = currTime;
        }else {
            correction[0] = 0;
        }
        return correction;
    }

    public void setEachPower(double lf, double rf, double lb, double rb, boolean halfspeed){
        if (halfspeed){
            lf /= 2;
            rf /= 2;
            lb /= 2;
            rb /= 2;
        }
        LF.setPower(lf);
        RF.setPower(rf);
        LB.setPower(lb);
        RB.setPower(rb);
    }

    public void setMotorPowers(double leftPower, double rightPower) {
        LF.setPower(Range.clip(leftPower, -1, 1));
        RF.setPower(Range.clip(rightPower, -1, 1));
        LB.setPower(Range.clip(leftPower, -1, 1));
        RB.setPower(Range.clip(rightPower, -1, 1));
    }

    public void setEachMotorPowers(double lf, double rf, double lb, double rb, boolean halfspeed) {
        if (halfspeed){
            lf /= 2;
            rf /= 2;
            lb /= 2;
            rb /= 2;
        }

        double min = 0.2;

        if (lf < 0)
            LF.setPower(Range.clip(lf, -1, -min));
        else if (lf > 0)
            LF.setPower(Range.clip(lf, min, 1));
        else{
            LF.setPower(0);
        }

        if (rf < 0)
            RF.setPower(Range.clip(rf, -1, -min));
        else if (rf > 0)
            RF.setPower(Range.clip(rf, min, 1));
        else{
            RF.setPower(0);
        }

        if (lb < 0)
            LB.setPower(Range.clip(lb, -1, -min));
        else if (lb > 0)
            LB.setPower(Range.clip(lb, min, 1));
        else{
            LB.setPower(0);
        }

        if (rb < 0)
            RB.setPower(Range.clip(rb, -1, -min));
        else if (rb > 0)
            RB.setPower(Range.clip(rb, min, 1));
        else{
            RB.setPower(0);
        }

    }

    public void setStrafePowers(double power, boolean right){
        if (right){
            LF.setPower(-power);
            RF.setPower(-power);
            LB.setPower(power);
            RB.setPower(power);
        }else {
            LF.setPower(power);
            RF.setPower(power);
            LB.setPower(-power);
            RB.setPower(-power);
        }
    }

    public void setMode(DcMotor.RunMode runMode) throws InterruptedException {
        LF.setMode(runMode);
        idle();
        RF.setMode(runMode);
        idle();
        LB.setMode(runMode);
        idle();
        RB.setMode(runMode);
        idle();
    }

    public void StrafetoPosition(double power, double tarX, double tarY, double tarheading) {  // Garrett(9/13/19) edited
        //Declare variables
        double min = 0.3;
        double curX;
        double tarHead = tarheading;
        double curHead;
        double RFpower = power;
        double RBpower = power;
        turnPIDV(tarHead, 0.4, 0, 0, false);    //turn towards the correct heading
        while (opModeIsActive() && (Math.abs(tarX - getRobotX()) > 0) || (Math.abs(tarY - getRobotY()) > 0)) {  //Move until at target position

            curHead = getRobotHeading();
            curX = getRobotX();

            if (curX < tarX) {  //If curX < tarX strafe right
                if (curHead >= tarHead + 0.5) { //If robot turning too far left
                    RBpower -= .05 * (curHead - tarHead);
                }
                else if (curHead <= tarHead - 0.5) { //If robot turning too right left
                    RFpower -= .05 * (tarHead - curHead);
                }
                LF.setPower(-Range.clip(power, min, 1));
                RF.setPower(Range.clip(-RFpower, min, 1));
                LB.setPower(Range.clip(-power, min, 1));
                RB.setPower(-Range.clip(RBpower, min, 1));
            } else {    //If curX > tarX strafe left
                if (curHead >= tarHead + 0.5) { //If robot turning too far left
                    RFpower -= .05 * (curHead - tarHead);
                }
                if (curHead <= tarHead - 0.5) { //If robot turning too far left
                    RBpower -= .05 * (tarHead - curHead);
                }
                LF.setPower(Range.clip(-power, min, 1));
                RF.setPower(-Range.clip(RFpower, min, 1));
                LB.setPower(-Range.clip(power, min, 1));
                RB.setPower(Range.clip(-RBpower, min, 1));
            }
        }
        stopMotors();
    }

    public void stopMotors(){
        setMotorPowers(0,0);
    }

    public int getEncoderAvg(){
        //divided by three for now because RF encoder returns wack values
        int avg = (Math.abs(LF.getCurrentPosition()) + Math.abs(LB.getCurrentPosition()) + Math.abs(RB.getCurrentPosition()))/3;
        //MAKE SURE ALL ENCODERS WORK BEFORE DIVIDING BY 4 OR PUT IN A CONDITION TO DETERMINE IF AN ENCODER VALUE IS WEIRD, AND DIVIDE ACCORDINGLY
        return avg;
    }

    public void resetEncoders(){
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();

    }

    public void resetLift(){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();
    }

    public void resetArm(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();
        /*arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        idle();*/
    }

    public void driveDistance(double power, double distance) throws InterruptedException {

        double total = distance * encoderToInches;
        double remaining, finalPower;
        ElapsedTime t = new ElapsedTime();
        t.reset();
        resetEncoders();

        while (opModeIsActive() && !isStopRequested() && getEncoderAvg() < distance * encoderToInches && t.seconds() < 10){
            remaining = total - getEncoderAvg();
            finalPower = (remaining/total) * power;
            //put in range clip if necessary
            setEachMotorPowers(finalPower,finalPower,finalPower,finalPower,false);
            telemetry.addData("target:", distance*encoderToInches);
            telemetry.addData("avg:", getEncoderAvg());
            telemetry.addData("LF Power", LF.getPower() + " " + LF.getCurrentPosition());
            telemetry.addData("RF Power", RF.getPower() + " " + RF.getCurrentPosition());
            telemetry.addData("LB Power", LB.getPower() + " " + LB.getCurrentPosition());
            telemetry.addData("RB Power", RB.getPower() + " " + RB.getCurrentPosition());
            telemetry.update();
        }
        stopMotors();
    }

    public void driveDistanceInc(double power, double distance) throws InterruptedException{

        double total = distance * encoderToInches;
        double remaining, finalPower = 0.2, prevTime = 0.0;
        ElapsedTime t = new ElapsedTime();
        t.reset();
        resetEncoders();

        while (opModeIsActive()&& !isStopRequested() && getEncoderAvg() < distance * encoderToInches && t.seconds() < 10){
            remaining = total - getEncoderAvg();
            if(t.milliseconds() - prevTime >= 100){
                prevTime = t.milliseconds();
                finalPower += 0.075;
            }
            //put in range clip if necessary
            finalPower = Range.clip(finalPower,0.2, power);
            setEachMotorPowers(finalPower,finalPower,finalPower,finalPower,false);

        }
        stopMotors();
    }

    public void driveAdjust(double tHeading, double power, double distance, int timeout){
        // ORIENTATION 0 TO 360
        // INCREASING GOING COUNTER CLOCKWISE (left)

        double total = distance * encoderToInches;
        double remaining, finalPower = power, origHeading = tHeading, error = 0, lp, rp;
        ElapsedTime t = new ElapsedTime();
        t.reset();
        resetEncoders();

        while (opModeIsActive() && !isStopRequested() && getEncoderAvg() < distance * encoderToInches && t.seconds() < timeout) {
            remaining = total - getEncoderAvg();

            error = origHeading - getYaw();
            if(error > 180){
                error = -(error-360);
            } else if (error < -180)
                error = 360+error;

            /*error = tHeading - get180Yaw(); //GET ANGLE REMAINING TO TURN (tANGLE MEANS TARGET ANGLE, AS IN THE ANGLE YOU WANNA GO TO)

            if(error > 180){
                error = -(error-180);
            }else  if(error < -180){
                error = -(error+180);
            }*/

            double m = 1.5;

            finalPower = (remaining / total) * power;

            if (power > 0) {
                if (error > 1) {
                    rp = finalPower*m;
                    lp = finalPower;
                } else if (error < -1) {
                    rp = finalPower;
                    lp = finalPower*m;
                } else {
                    rp = finalPower;
                    lp = finalPower;
                }
                rp = Range.clip(rp,0.2,1);
                lp = Range.clip(lp, 0.2,1);
            } else {
                if (error > 1) {
                    rp = finalPower;
                    lp = finalPower*m;
                } else if (error < -1) {
                    rp = finalPower*m;
                    lp = finalPower;
                } else {
                    rp = finalPower;
                    lp = finalPower;
                }
                rp = Range.clip(rp,-1,-0.2);
                lp = Range.clip(lp, -1,-0.2);
            }
            setMotorPowers(lp,rp);
            telemetry.addData("left power: ", lp);
            telemetry.addData("right power: ", rp);
            telemetry.addData("error", error);
            telemetry.addData("current angle", getYaw());
            telemetry.addData("target angle", tHeading);
            telemetry.update();

        }

        stopMotors();
    }

    public void driveAdjustO(double tHeading, double power, double distance, int timeout){
        // ORIENTATION 0 TO 360
        // INCREASING GOING COUNTER CLOCKWISE

        double total = distance * encoderToInches;
        double remaining, finalPower = power, error = 0, lp, rp;
        int[] encVals = new int[4];
        ElapsedTime t = new ElapsedTime();
        t.reset();
        resetEncoders();

        while (opModeIsActive() && !isStopRequested() && getEncoderAvg() < distance * encoderToInches && t.seconds() < timeout) {
            remaining = total - getEncoderAvg();

            error = tHeading - get180Yaw(); //GET ANGLE REMAINING TO TURN (tANGLE MEANS TARGET ANGLE, AS IN THE ANGLE YOU WANNA GO TO)

            if(error > 180){
                error = -(error-180);
            }else  if(error < -180){
                error = -(error+180);
            }

            //finalPower = (remaining / total) * power;

            if (power > 0) {
                if (error > 1) {
                    rp = finalPower*1.5;
                    lp = finalPower;
                } else if (error < -1) {
                    rp = finalPower;
                    lp = finalPower*1.5;
                } else {
                    rp = finalPower;
                    lp = finalPower;
                }
                rp = Range.clip(rp,0.2,1);
                lp = Range.clip(lp, 0.2,1);
            } else {
                if (error > 1) {
                    rp = finalPower;
                    lp = finalPower*1.5;
                } else if (error < -1) {
                    rp = finalPower*1.5;
                    lp = finalPower;
                } else {
                    rp = finalPower;
                    lp = finalPower;
                }
                rp = Range.clip(rp,-1,-0.2);
                lp = Range.clip(lp, -1,-0.2);
            }
            setMotorPowers(lp,rp);
            telemetry.addData("left power: ", lp);
            telemetry.addData("right power: ", rp);
            telemetry.addData("error", error);
            telemetry.addData("current angle", getYaw());
            telemetry.addData("target angle", tHeading);
            telemetry.update();

        }

        stopMotors();
    }

    public void driveAdjustS(double tHeading, double power, double distance, int timeout){

        double total = distance * encoderToInches;
        double remaining, finalPower = power, origHeading = tHeading, error = 0, lp, rp;
        ElapsedTime t = new ElapsedTime();
        t.reset();
        resetEncoders();

        while (opModeIsActive() && !isStopRequested() && getEncoderAvg() < distance * encoderToInches && t.seconds() < 10) {
            remaining = total - getEncoderAvg();
            error = origHeading - getYaw();
            finalPower = (remaining / total) * power;

            if (power > 0) {
                if (error > 1) {
                    rp = finalPower*1.2;
                    lp = 0.8*finalPower;
                } else if (error < -1) {
                    rp = finalPower*0.8;
                    lp = finalPower*1.2;
                } else {
                    rp = finalPower;
                    lp = finalPower;
                }
                rp = Range.clip(rp,0.2,1);
                lp = Range.clip(lp, 0.2,1);
            } else {
                if (error > 1) {
                    rp = finalPower*0.8;
                    lp = finalPower*1.2;
                } else if (error < -1) {
                    rp = finalPower*1.2;
                    lp = 0.8*finalPower;
                } else {
                    rp = finalPower;
                    lp = finalPower;
                }
                rp = Range.clip(rp,-1,-0.2);
                lp = Range.clip(lp, -1,-0.2);
            }
            setMotorPowers(lp,rp);
            telemetry.addData("left power: ", lp);
            telemetry.addData("right power: ", rp);
            telemetry.addData("error", error);
            telemetry.update();

        }

        stopMotors();
    }

    /*public void strafeDistance(double power, double distance, boolean right) throws InterruptedException{
        resetEncoders();
        double minP = 0.25;
        double actualP = minP;
        double sTime = runtime.milliseconds();
        double cTime = runtime.milliseconds();
        while (opModeIsActive() && getEncoderAvg() < distance * 55 && !isStopRequested()){
            sTime = runtime.milliseconds();
            if(actualP < power && cTime + 200 >= sTime) {
                actualP += .05;
                cTime = runtime.milliseconds();
                Range.clip(actualP, 0, power);
            }*//*
            if (!right){
                LF.setPower(-actualP);
                RF.setPower(-actualP);
                LB.setPower(actualP);
                RB.setPower(actualP);
            }else {
                LF.setPower(actualP);
                RF.setPower(actualP);
                LB.setPower(-actualP);
                RB.setPower(-actualP);
            }
            telemetry.addData("target:", distance*55);
            telemetry.addData("avg:", getEncoderAvg());
            telemetry.addData("LF Power", LF.getPower() + " " + LF.getCurrentPosition());
            telemetry.addData("RF Power", RF.getPower() + " " + RF.getCurrentPosition());
            telemetry.addData("LB Power", LB.getPower() + " " + LB.getCurrentPosition());
            telemetry.addData("RB Power", RB.getPower() + " " + RB.getCurrentPosition());
            telemetry.update();
        }
        stopMotors();
    }*/

    public void strafeDistance(double power, double distance, boolean right) throws InterruptedException{
        resetEncoders();
        while (opModeIsActive() && getEncoderAvg() < distance * 100 && !isStopRequested()){

            if (right){
                setEachMotorPowers(-power,-power,power,power,false);
            }else {
                setEachMotorPowers(power,power,-power,-power,false);
            }
            telemetry.addData("target:", distance*55);
            telemetry.addData("avg:", getEncoderAvg());
            telemetry.addData("LF Power", LF.getPower() + " " + LF.getCurrentPosition());
            telemetry.addData("RF Power", RF.getPower() + " " + RF.getCurrentPosition());
            telemetry.addData("LB Power", LB.getPower() + " " + LB.getCurrentPosition());
            telemetry.addData("RB Power", RB.getPower() + " " + RB.getCurrentPosition());
            telemetry.update();
        }
        stopMotors();
    }

    /*
    public void driveAdjust(double power, double distance) throws InterruptedException{
        double setHeading = getYaw();
        double error = 0;
        double correction = 0;
        double leftPower = 0, rightPower = 0;
        resetEncoders();

        while (opModeIsActive() && !isStopRequested() && getEncoderAvg() < distance * encoderToInches){
            error = getYaw() - setHeading;
            //Right now, getYaw() returns -180 to +180
            if(Math.abs(error) > 2){
                correction = error * 0.1;
                leftPower -= correction;
                rightPower += correction;
            }else{
                leftPower = power;
                rightPower = power;
            }
            setMotorPowers(leftPower, rightPower);
        }
        stopMotors();
    }

     */

    public void strafeAdjust(double power, double distance, double tHeading, boolean right) throws InterruptedException{
        // ORIENTATION 0 TO 360
        // INCREASING GOING COUNTER CLOCKWISE
        double total = distance * encoderToInches * 2.7;
        double remaining, finalPower = power, error;
        ElapsedTime t = new ElapsedTime();
        t.reset();
        resetEncoders();


        while (opModeIsActive() && !isStopRequested() && getEncoderAvg() < total && t.seconds() < 10) {
            remaining = total - getEncoderAvg();
            //error = getYaw()-tHeading;
            //if(error > 180) error = -(error-360);
            //else if (error < -180)
            //    error = error + 360;

            error = tHeading - getYaw();
            if(error > 180){
                error = -(error-360);
            } else if (error < -180)
                error = 360+error;

            double p = 0.8;
            if (right) {
                if (error > 1) {
                    setEachMotorPowers(-finalPower,-finalPower,finalPower,p*finalPower,false); //check
                } else if (error < -1) {
                    setEachMotorPowers(-finalPower,p*-finalPower,finalPower,finalPower,false); //check
                } else {
                    setEachMotorPowers(-finalPower,-finalPower,finalPower,finalPower,false); //check
                }
            } else {
                if (error > 1) {
                    setEachMotorPowers(finalPower,finalPower,p*-finalPower,-finalPower,false); //check
                } else if (error < -1) {
                    setEachMotorPowers(p*finalPower,finalPower,-finalPower,-finalPower,false); //check
                } else {
                    setEachMotorPowers(finalPower,finalPower,-finalPower,-finalPower,false); //check
                }
            }
            telemetry.addData("target:", distance*55);
            telemetry.addData("avg:", getEncoderAvg());
            telemetry.addData("LF:", LF.getPower());
            telemetry.addData("LB:", LB.getPower());
            telemetry.addData("RF:", RF.getPower());
            telemetry.addData("RB:", RB.getPower());
            telemetry.addData("error", error);
            telemetry.update();
        }
        stopMotors();
    }

    public void driveForward(double x, double y, double power, double trgtHead){
        // Angle adjustment while driving to a specific point
        // TO DO: calculate proportional error to decrease power more if robot angle is larger

        double origError, error, errorAdjust = 1;

        updateRobotPosition();
        origError = trgtHead - getRobotHeading();

        while (opModeIsActive() && (Math.abs(x - getRobotX()) > 5) || (Math.abs(y - getRobotY()) > 5))  {
            updateRobotPosition();
            error = trgtHead - getRobotHeading();
            errorAdjust = error/origError;
            if (trgtHead - getRobotHeading() > 1)
                setMotorPowers(power, power * errorAdjust); // default error is 0.8
            else if (trgtHead - getRobotHeading() < -1)
                setMotorPowers(power * errorAdjust, power);
            else
                setMotorPowers(power, power);

            if (Math.abs(trgtHead - getRobotHeading()) > 30) // stops robot if robot is turned too off course
                break;
        }

        /*

        original foundational method - mindy

        while (opModeIsActive() && (Math.abs(x - getRobotX()) > 0) || (Math.abs(y - getRobotY()) > 0))  {

            if (trgtHead - getRobotHeading() > 1)
                setMotorPowers(power, power * 0.8); // default error is 0.8
            else if (trgtHead - getRobotHeading() < -1)
                setMotorPowers(power * 0.8, power);
            else
                setMotorPowers(power, power);

            if (Math.abs(trgtHead - getRobotHeading()) > 30) // stops robot if robot is turned too off course
               break;
        }
         */

        stopMotors();
        telemetry.addData("Target: ", x + " , " + y);
        telemetry.update();
    }

    public void driveToPoint(double power, double xTarget, double yTarget) {
        updateRobotPosition();
        double curX = getRobotX();
        double curY = getRobotY();

        double trgtX = xTarget;
        double trgtY = yTarget;

        double diffX = trgtX - curX;
        double diffY = trgtY - curY;

        double curHeading = getRobotHeading(); //Assuming 0-360
        double diffHeading = Math.abs(Math.toDegrees(Math.atan(diffX / diffY))); //CALCULATE ANGLE FROM CURRENT COORD TO TARGET COORD
        double trgtHeading;

        //ACCOUNT FOR ANGLES IN ALL FOUR QUADRANTS
        if (diffX > 0 && diffY > 0)
            trgtHeading = diffHeading;
        else if (diffX < 0 && diffY > 0)
            trgtHeading = 360 - diffHeading;
        else if (diffX < 0 && diffY < 0)
            trgtHeading = 180 + diffHeading;
        else if (diffX > 0 && diffY < 0)
            trgtHeading = 180 - diffHeading;
        else
            trgtHeading = 0;

        turnPIDV(trgtHeading, 0.4, 0, 0, (trgtHeading - curHeading > 180));
        driveForward(trgtX, trgtY, power, trgtHeading);
        stopMotors();
    }

    public void turnPIDV(double tAngle, double kP, double kI, double kD, boolean flip){

        double power, prevError, error, dT, prevTime, currTime, P, I, D; //DECLARE ALL VARIABLES

        prevError = error = tAngle - getRobotHeading(); //INITIALIZE THESE VARIABLES

        power = dT = prevTime = currTime = P = I = D = 0;

        ElapsedTime time = new ElapsedTime(); //CREATE NEW TIME OBJECT
        resetTime();
        while (opModeIsActive() && Math.abs(error) > 0.5){ //put timeout in later
            updateRobotPosition();
            prevError = error;
            error = tAngle - getRobotHeading(); //GET ANGLE REMAINING TO TURN (tANGLE MEANS TARGET ANGLE, AS IN THE ANGLE YOU WANNA GO TO)
            prevTime = currTime;
            currTime = time.milliseconds();
            dT = currTime - prevTime; //GET DIFFERENCE IN CURRENT TIME FROM PREVIOUS TIME
            P = error;
            I = error * dT;
            D = (error - prevError)/dT;
            power = P * kP + I * kI + D * kD;

            if (flip)
                power *= -1;

            //Basically, tell the robot to take the shortest way to the target angle.
            //For example, if robot is at 359 and target angle is 5 degrees, move 6 deg clockwise instead
            //of 354 deg counter clockwise

            if(power < 0.2){
                power = 0.2;
            }else if(power > -0.2){
                power = -0.2;
            }

            setMotorPowers(power, -power);

            telemetry.addData("tAngle: ", tAngle)
                    .addData("P:", P)
                    .addData("I:", I)
                    .addData("D:", D)
                    .addData("power", power)
                    .addData("error: ", error)
                    .addData("currTime: ", currTime);
            telemetry.update();
        }
        stopMotors();
    }

    public void driveTime(double power, double seconds){
        setMotorPowers(power, power);
        long time = (long)(seconds) * 1000;
        sleep(time);
        stopMotors();
    }

    //MANIP METHODS

    public void setClawPosition(boolean open){
        if (open){
            claw.setPosition(clawEndPosition);
        }
        else{
            claw.setPosition(clawStartPosition);
        }
    }

    public void grabStone(int pos, boolean back){
        switch(pos){
            case -1:
                foundationL.setPosition(0);
                break;
            case 0:
                foundationL.setPosition(0);
                break;
            case 1:
                if (!back)
                    foundationR.setPosition(1);
                else
                    foundationL.setPosition(0);
                break;
        }
        sleep(1000);
    }

    public void grabStoneBlue(int pos, boolean back){
        switch(pos){
            case -1:
                foundationL.setPosition(0);
                break;
            case 0:
                foundationR.setPosition(1);
                break;
            case 1:
                if (!back)
                    foundationR.setPosition(1);
                else
                    foundationL.setPosition(0);
                break;
        }
        sleep(1000);
    }

    public void setArmPosition(int position) {
        arm.setTargetPosition(position);
        int c = arm.getCurrentPosition();
        while (opModeIsActive() && !(c < position + 10 && c > position - 10)) {
            if (c < position - 10) {
                if (c > 150)
                    arm.setPower(0.1);
                else
                    arm.setPower(0.6);
            }
            if (c > position + 10) {
                if (c > 250)
                    arm.setPower(-0.6);
                else
                    arm.setPower(-0.1);
            }
        }
        //arm.setTargetPosition(position);
    }

    public void setArmPositionP(int position){
        arm.setTargetPosition(position);
        int c = arm.getCurrentPosition();
        double prevError, slope, error = position - c;
        while (opModeIsActive() && !(c < position + 10 && c > position - 10)){
             prevError = error;
             error = position - c;
             slope = error/prevError;
             if (slope > 6)
            arm.setPower(error * 6);
        }
    }

    public void armSmallUp() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double error = 25 - arm.getCurrentPosition();
        ElapsedTime time = new ElapsedTime();
        double P, D, prevE, pTime = 0.0;
        while( opModeIsActive() && !isStopRequested() && Math.abs(error) > 1){
            prevE = error;
            error = 25 - arm.getCurrentPosition();

            P = (0.3) * error;
            D = (error - prevE) / (time.milliseconds() - pTime) * (0.002 / 25);
            arm.setPower(-(P + D));

            pTime = time.milliseconds();
        }
        arm.setPower(0);
    }

    public void foundationD( boolean deployed){
        if (deployed){
            stickL.setPosition(0);
            stickR.setPosition(1);
        }
        else {
            stickL.setPosition(1);
            stickR.setPosition(0);
        }
        //DON'T ADD A SLEEP!!!
    }
    //1 = right when servo testing
    public void hook( boolean left, boolean right)
    {
        if(left)
            foundationL.setPosition(0);
        else
            foundationL.setPosition(1);
        if(right)
            foundationR.setPosition(1);
        else
            foundationR.setPosition(0);
    }

    public void fang( boolean left, boolean right)
    {
        if(left)
            fangL.setPosition(0);
        else
            fangL.setPosition(1);
        if(right)
            fangR.setPosition(1);
        else
            fangR.setPosition(0);
    }

    /*public void setArm(int target, double pwr){
        double endPos = target; //GET VALUE RANGE IN TELEOP : 0 - ?
        double startPos = arm.getCurrentPosition();
        double power = 0;

        while (opModeIsActive()&& !isStopRequested() && Math.abs(target-arm.getCurrentPosition()) > 3){ //3 TICKS MARGIN OF ERROR
            if(arm.getCurrentPosition() < target){
                if(arm.getCurrentPosition() < 220){ //VALUE OF HIGHEST POINT
                    power = -0.3;
                }else{
                    power = -0.1;
                }
            }else if(arm.getCurrentPosition() > target){
                if(arm.getCurrentPosition() < 220){ //VALUE OF HIGHEST POINT
                    power = 0.1;
                }else{
                    power = 0.3;
                }
            }

            power = pwr;
            arm.setTargetPosition(target);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(power);
            while (opModeIsActive()&& !isStopRequested() && arm.isBusy()) {
                telemetry.addData("arm encoder:", arm.getCurrentPosition());
                telemetry.addData("arm power:", power);
                telemetry.update();
             //   sleep(250);
            }
    }*/

    //ARM MOVEMENT

    /*public void armSetPositition(int targetValue, double armPwr){
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(targetValue);
        arm.setPower(Range.clip(armPwr, 0, 1));
        while(arm.isBusy())
        {

        }
        arm.setPower(0);
    }*/
    /*public void armPID(double value, double P, double I, double D, double timeOut) {

        double power, prevError, error, dT, prevTime, currTime; //DECLARE ALL VARIABLES

        double kP = P;
        double kI = I;
        double kD = D;

        prevError = error = value - getEncoderAvg(); //INITIALIZE THESE VARIABLES

        power = dT = prevTime = currTime = 0.0;

        ElapsedTime time = new ElapsedTime(); //CREATE NEW TIME OBJECT
        resetTime();
        while (opModeIsActive() && Math.abs(error) > 0.7 && currTime < timeOut){
            prevError = error;

            error = value - getEncoderAvg();

            prevTime = currTime;
            currTime = time.milliseconds();
            dT = currTime - prevTime; //GET DIFFERENCE IN CURRENT TIME FROM PREVIOUS TIME
            power = (error * kP) + (error * dT * kI) + ((error - prevError)/dT * kD);

            arm.setPower(Range.clip(power, 0.2, 1));
        }
    }
    */
    //TURN METHODS

    public void turnPID(double tAngle, double P, double I, double D, double timeOut){
        // ORIENTATION -180 TO 180
        // - is right, + is left
        // constants P: 0.6/180    I: 0.0001   D: 0.5
        double power, prevError, error, dT, prevTime, currTime; //DECLARE ALL VARIABLES

        double kP = P;
        double kI = I;
        double kD = D;

        prevError = error = tAngle - get180Yaw(); //INITIALIZE THESE VARIABLES

        power = dT = prevTime = currTime = 0.0;

        ElapsedTime time = new ElapsedTime(); //CREATE NEW TIME OBJECT
        resetTime();
        while (opModeIsActive() && Math.abs(error) > 0.7 && currTime < timeOut){
            prevError = error;
            error = tAngle - get180Yaw(); //GET ANGLE REMAINING TO TURN (tANGLE MEANS TARGET ANGLE, AS IN THE ANGLE YOU WANNA GO TO)

            if(error > 180){
                error = -(error-180);
            }else  if(error < -180){
                error = -(error+180);
            }

            prevTime = currTime;
            currTime = time.milliseconds();
            dT = currTime - prevTime; //GET DIFFERENCE IN CURRENT TIME FROM PREVIOUS TIME
            power = (error * kP) + ((error) * dT * kI) + ((error - prevError)/dT * kD);

            if (power < 0)
                setMotorPowers(Range.clip(-power, 0.2, 0.5), Range.clip(power, -0.5, -0.2));
            else
                setMotorPowers(Range.clip(-power, -0.5, -0.2), Range.clip(power, 0.2, 0.5));

            telemetry.addData("tAngle: ", tAngle)
                    .addData("currAngle: ", get180Yaw())
                    .addData("kP:", error * kP)
                    .addData("kI:", error * dT * kI)
                    .addData("kD:", (error - prevError)/dT * kD)
                    .addData("power", power)
                    .addData("ACTUAL POWER:",LF.getPower())
                    .addData("error: ", error)
                    .addData("currTime: ", currTime);
            telemetry.update();
        }
        stopMotors();
    }

    public void turnPIDF(double tAngle, double P, double I, double D, double timeOut){
        // ORIENTATION -180 TO 180
        // - is right, + is left
        // increased minimum power in order to push foundation all the flush to the wall
        // constants P: .8/90   I: .0001    D: 2
        double power, prevError, error, dT, prevTime, currTime; //DECLARE ALL VARIABLES

        double kP = P;
        double kI = I;
        double kD = D;

        prevError = error = tAngle - get180Yaw(); //INITIALIZE THESE VARIABLES

        power = dT = prevTime = currTime = 0.0;

        ElapsedTime time = new ElapsedTime(); //CREATE NEW TIME OBJECT
        resetTime();
        while (opModeIsActive() && Math.abs(error) > 0.7 && currTime < timeOut){
            prevError = error;
            error = tAngle - get180Yaw(); //GET ANGLE REMAINING TO TURN (tANGLE MEANS TARGET ANGLE, AS IN THE ANGLE YOU WANNA GO TO)

            if(error > 180){
                error = -(error-180);
            }else  if(error < -180){
                error = -(error+180);
            }

            prevTime = currTime;
            currTime = time.milliseconds();
            dT = currTime - prevTime; //GET DIFFERENCE IN CURRENT TIME FROM PREVIOUS TIME
            power = (error * kP) + ((error) * dT * kI) + ((error - prevError)/dT * kD);

            if (power < 0)
                setMotorPowers(Range.clip(-power, 0.3, 0.4), Range.clip(power, -0.4, -0.3));
            else
                setMotorPowers(Range.clip(-power, -0.4, -0.3), Range.clip(power, 0.3, 0.4));

            foundationD(true);

            telemetry.addData("tAngle: ", tAngle)
                    .addData("currAngle: ", get180Yaw())
                    .addData("kP:", error * kP)
                    .addData("kI:", error * dT * kI)
                    .addData("kD:", (error - prevError)/dT * kD)
                    .addData("power", power)
                    .addData("ACTUAL POWER:",LF.getPower())
                    .addData("error: ", error)
                    .addData("currTime: ", currTime);
            telemetry.update();
        }
        stopMotors();
    }

    public void turnArc(double tAngle, double P, double I, double D, boolean right, double timeOut){
        // ORIENTATION -180 TO 180
        // - is right, + is left
        // increased minimum power in order to push foundation all the flush to the wall
        // right boolean refers to what side will turn
        // constants P: 0.8/90      I: 0.0001       D: 2
        double power, prevError, error, dT, prevTime, currTime; //DECLARE ALL VARIABLES

        double kP = P;
        double kI = I;
        double kD = D;

        prevError = error = tAngle - get180Yaw(); //INITIALIZE THESE VARIABLES

        power = dT = prevTime = currTime = 0.0;

        ElapsedTime time = new ElapsedTime(); //CREATE NEW TIME OBJECT
        resetTime();
        while (opModeIsActive() && Math.abs(error) > 0.7 && currTime < timeOut){
            prevError = error;
            error = tAngle - get180Yaw(); //GET ANGLE REMAINING TO TURN (tANGLE MEANS TARGET ANGLE, AS IN THE ANGLE YOU WANNA GO TO)

            if(error > 180){
                error = -(error-180);
            }else  if(error < -180){
                error = -(error+180);
            }

            prevTime = currTime;
            currTime = time.milliseconds();
            dT = currTime - prevTime; //GET DIFFERENCE IN CURRENT TIME FROM PREVIOUS TIME
            power = (error * kP) + ((error) * dT * kI) + ((error - prevError)/dT * kD);

            if (right) {
                if (power < 0) // face left
                    setMotorPowers(0, Range.clip(power, -1, -0.4));
                else
                    setMotorPowers(0, Range.clip(power, 0.4, 1));
            } else {
                if (power < 0) // face left
                    setMotorPowers(Range.clip(-power, 0.4, 1), 0);
                else
                    setMotorPowers(Range.clip(-power, -1, -0.4), 0);
            }

            telemetry.addData("tAngle: ", tAngle)
                    .addData("currAngle: ", get180Yaw())
                    .addData("kP:", error * kP)
                    .addData("kI:", error * dT * kI)
                    .addData("kD:", (error - prevError)/dT * kD)
                    .addData("power", power)
                    .addData("ACTUAL POWER:",LF.getPower())
                    .addData("error: ", error)
                    .addData("currTime: ", currTime);
            telemetry.update();
        }
        stopMotors();
    }

    public void turnPIDtest(double tAngle, double P, double I, double D, double timeOut){

        double power, prevError, error, dT, prevTime, currTime; //DECLARE ALL VARIABLES

        ArrayList<Double> amps = new ArrayList<>();
        ArrayList<Double> times = new ArrayList<>();

        double kP = P;
        double kI = I;
        double kD = D;

        prevError = error = tAngle - getYaw(); //INITIALIZE THESE VARIABLES

        power = dT = prevTime = currTime = 0.0;

        ElapsedTime time = new ElapsedTime(); //CREATE NEW TIME OBJECT
        resetTime();
        while (opModeIsActive() && Math.abs(error) > 0.5 && currTime < timeOut){
            prevError = error;
            error = tAngle - getYaw(); //GET ANGLE REMAINING TO TURN (tANGLE MEANS TARGET ANGLE, AS IN THE ANGLE YOU WANNA GO TO)
            if(error > 180){
                error = -(360-error);
            }
            prevTime = currTime;
            currTime = time.milliseconds();
            dT = currTime - prevTime; //GET DIFFERENCE IN CURRENT TIME FROM PREVIOUS TIME


            if(power > 0){
                power = Range.clip(power, 0.2,1);
            }else if (power < 0){
                power = Range.clip(power, -1,-0.2);
            }

            setMotorPowers(power,-power);

            amps.add(Math.abs(error));
            times.add(currTime);

            telemetry.addData("tAngle: ", tAngle)
                    .addData("currAngle: ", getYaw())
                    .addData("kP:", error * kP)
                    .addData("kI:", error * dT * kI)
                    .addData("kD:", (error - prevError)/dT * kD)
                    .addData("power", power)
                    .addData("error: ", error)
                    .addData("currTime: ", currTime);
            telemetry.update();
        }
        stopMotors();

       /** ArrayList<Double> maxes = new ArrayList<>();
        ArrayList<Double> maxTimes = new ArrayList<>();

        for(int a = 1; a < amps.size()-1; a++){
            if(amps.get(a) > amps.get(a-1) && amps.get(a) > amps.get(a+1)){
                maxes.add(amps.get(a));
                maxTimes.add(times.get(a));
            }
        }

        telemetry.addData("maxes: ", maxes.toString());
        telemetry.addData("times: ", maxTimes.toString());
        telemetry.update();
        sleep(10000);**/
    }

    public void turnTime(double power, boolean right, long seconds){
        if (right)
            setMotorPowers(power, -power);
        else
            setMotorPowers(-power, power);

        sleep(seconds);
        stopMotors();
    }

    public void rotate(double targetAngleChange, int timeout) {

        runtime.reset();

        double power = 0;
        double origDiff = getYaw() - targetAngleChange;
        double deltaHeading = 0;

        while (opModeIsActive() && (Math.abs(getYaw()-targetAngleChange) > 1) && (runtime.seconds() < timeout)) {

            telemetry.addData("Turning:", "From " + getYaw() + " to " + targetAngleChange);
            telemetry.update();

            deltaHeading = getYaw() - targetAngleChange; //GET ANGLE LEFT UNTIL TARGET ANGLE
            power = Range.clip(0.4 * deltaHeading/origDiff, 0.2, 1); //PROPORTIONAL SPEED
            /** Why is dHeading/oDiff multiplied by 0.4? -Garrett **/
            if (deltaHeading < -180 || (deltaHeading > 0 && deltaHeading < 180) ) { //LEFT IS + , RIGHT IS -
                setMotorPowers(power, -power);
            } else {
                setMotorPowers(-power, power);
            }
        }
        stopMotors();
    }

    public void arcT (double targetAngle, double speed, double timeout)
    {
        ElapsedTime time = new ElapsedTime(); //CREATE NEW TIME OBJECT
        resetTime();
        double error = targetAngle - get180Yaw(); //GET ANGLE REMAINING TO TURN (tANGLE MEANS TARGET ANGLE, AS IN THE ANGLE YOU WANNA GO TO)
        while(opModeIsActive() && error > 0.05 && time.seconds() < timeout)
        {
            error = targetAngle - get180Yaw(); //GET ANGLE REMAINING TO TURN (tANGLE MEANS TARGET ANGLE, AS IN THE ANGLE YOU WANNA GO TO)

        }
    }

    //TIME METHODS

    public void resetTime(){
        runtime.reset();
    }

    public double getTime(){
        return runtime.seconds();
    }

    //VISION METHODS

    int pos = 0;

    public int getSkystonePos(){
        return pos;
    }

    public boolean updateRobotPosition(){

        if (!opModeIsActive()) return false;

        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() && trackable.getName() != "Stone Target") {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        if (targetVisible) {
            // express position (translation) of robot in inches.
            translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            telemetry.update();
            return true;
        }
        else {
            telemetry.addData("Visible Target", "none");
            telemetry.update();
            return false;
        }
    }

    public Bitmap getBitmap() throws InterruptedException {
        Bitmap bm = null;
        if(opModeIsActive()&& !isStopRequested()){
            frame = vuforiaPC.getFrameQueue().take();
            long num = frame.getNumImages();

            for(int i = 0; i < num; i++){
                if(frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565){
                    rgb = frame.getImage(i);
                }
            }

            bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(rgb.getPixels());
        }

        frame.close();

        return bm;
    }

    public void detectSkystone(Bitmap bm) throws InterruptedException {

        //set threshold for yellow or not yellow?
        int stonepos = 0;

        if (bm != null) {

            //figure out proper thresholds
            int redLim = 30;
            int greenLim = 30;
            int blueLim = 30;

            ArrayList<Integer> colorPix = new ArrayList<Integer>();

            for (int x = 0; x < bm.getHeight(); x++) {
                for (int y = 0; y < bm.getWidth(); y++) {
                    //previously (c,r)
                    if (red(bm.getPixel(y, x)) <= redLim && green(bm.getPixel(y, x)) <= greenLim && blue(bm.getPixel(y, x)) <= blueLim) {
                        colorPix.add(x); // previously it was adding c, so the column instead of row
                        //check once more if 480 is the x and 640 is y
                    }
                }
            }

            int sum = 0;
            for (Integer x : colorPix)
                sum += x;

            int avgX = 0, maxX = 0, minX =0;
            if(colorPix.size() != 0){
                avgX = sum / colorPix.size();

                maxX = Collections.max(colorPix);
                minX = Collections.min(colorPix);
            }


            if (avgX < 160) {
                stonepos = -1;
            } else if (avgX < 320) {
                stonepos = 0;
            } else {
                stonepos = 1;
            }

            telemetry.addData("bitmap width:", bm.getWidth()); //640
            telemetry.addData("bitmap height:", bm.getHeight()); //480 across I think?
            telemetry.addData("max x: ", maxX);
            telemetry.addData("min x: ", minX);
            telemetry.addData("x avg: ", avgX);
            //telemetry.addData("stonepos: ", stonepos);
            telemetry.update();
        }else{
            //change it to whatever is closest
            telemetry.addData("Bitmap null:", "Default center(?)");
            telemetry.update();
        }

        //return stonepos;
    }

    public void detectSkystoneCropped(Bitmap bm) throws InterruptedException {

        //set threshold for yellow or not yellow?
        int stonepos = 0;

        if (bm != null) {

            //figure out proper thresholds
            int redLim = 30;
            int greenLim = 30;
            int blueLim = 30;
            int pix;
            ArrayList<Integer> blackPix = new ArrayList<Integer>();

            for (int x = 0; x < 795; x++) {
                for (int y = bm.getHeight()/2; y < bm.getHeight(); y++) {
                    pix = bm.getPixel(x,y);
                    if(red(pix) < 25 && green(pix) < 25 && blue(pix) < 25){
                        blackPix.add(x);
                    }
                }
            }

            int sum = 0;
            for (Integer x : blackPix)
                sum += x;

            int avgX = 0, maxX = 0, minX =0;
            if(blackPix.size() != 0){
                avgX = sum / blackPix.size();

                maxX = Collections.max(blackPix);
                minX = Collections.min(blackPix);
            }


            if (avgX < 265) {
                stonepos = -1;
            } else if (avgX < 530) {
                stonepos = 0;
            } else {
                stonepos = 1;
            }

            telemetry.addData("bitmap width:", bm.getWidth()); //640
            telemetry.addData("bitmap height:", bm.getHeight()); //480 across I think?
            telemetry.addData("max x: ", maxX);
            telemetry.addData("min x: ", minX);
            telemetry.addData("x avg: ", avgX);
            telemetry.addData("black", blackPix.size());
            telemetry.addData("stonepos: ", stonepos);
            telemetry.update();
        }else{
            //change it to whatever is closest
            telemetry.addData("Bitmap null:", "Default center(?)");
            telemetry.update();
        }

        //return stonepos;
    }

    public int detectSkystoneOnePix(Bitmap bm, boolean red, int rx, int ry, int lx, int ly) throws InterruptedException {

        //set threshold for yellow or not yellow?
        int stonepos = 0;
        int leftPix, rightPix;

        if (bm != null) {

            //figure out proper thresholds
            int redLim = 30;
            lx = Range.clip(lx, 0, bm.getWidth()-1);
            ly = Range.clip(ly, 0, bm.getHeight()-1);
            rx = Range.clip(rx, 0, bm.getWidth()-1);
            ry = Range.clip(ry, 0, bm.getHeight()-1);

            if(red){
                leftPix = red(bm.getPixel(lx,ly));
                rightPix = red(bm.getPixel(rx,ry));
            }else{
                rightPix = red(bm.getPixel(960,550));
                leftPix = red(bm.getPixel(320,550));
            }

            if (Math.abs(leftPix - rightPix) >= 50) {
                if(leftPix < rightPix){
                    stonepos = 0;
                }else{
                    stonepos = 1;
                }
            }else{
                stonepos = -1;
            }

            telemetry.addData("rx: ", rx);
            telemetry.addData("ry: ", ry);
            telemetry.addData("lx: ", lx);
            telemetry.addData("ly: ", ly);
            telemetry.addData("bitmap width:", bm.getWidth()); //1280
            telemetry.addData("bitmap height:", bm.getHeight()); //720
            telemetry.addData("left pix red: ", leftPix);
            telemetry.addData("right pix red: ", rightPix);
            telemetry.addData("stonepos: ", stonepos);
            telemetry.update();
        }else{
            //change it to whatever is closest
            telemetry.addData("Bitmap null:", "Default center(?)");
            telemetry.update();
        }

        return stonepos;
    }

    public int detectSkystoneOnePix(Bitmap bm, boolean red) throws InterruptedException {

        //set threshold for yellow or not yellow?
        int stonepos = 0;
        int leftRed, midRed, rightRed;

        if (bm != null) {

            //figure out proper threshold

            //actually blue
            if(red){
                //USE ALL THREE OR JUST TWO?
                leftRed = red(bm.getPixel(1150, 250)); //originally 250 310
                midRed = red(bm.getPixel(860,250));
                rightRed = red(bm.getPixel(560,250));
            }else{ //GET PIXELS FOR BLUE LATER
                //actually red
                leftRed = red(bm.getPixel(850,250));
                midRed = red(bm.getPixel(470,250));//orig 490
                rightRed = red(bm.getPixel(300,250)); //originally x was 200 but no detecting correctly
            }

            ArrayList<Integer> pixels = new ArrayList<>();
            pixels.add(leftRed);
            pixels.add(midRed);
            pixels.add(rightRed);

            if(Collections.min(pixels) == leftRed){
                stonepos = -1;
            }else if(Collections.min(pixels) == midRed){
                stonepos = 0;
            }else if (Collections.min(pixels) == rightRed){
                stonepos = 1;
            }

            telemetry.addData("Reds Detected: ", pixels);
            telemetry.addData("stonepos: ", stonepos);
            telemetry.update();
            sleep(1000);
        }else{
            //change it to whatever is closest
            telemetry.addData("Bitmap null:", "Default center");
            telemetry.update();
        }

        return stonepos;
    }

    public void detectSkystoneNew(Bitmap bm) throws InterruptedException {

        //set threshold for yellow or not yellow?
        int stonepos = 0;

        if (bm != null && opModeIsActive() && !isStopRequested()) {

            /**
             * R: >200
             * G: <140
             * B: <50
             */

            //figure out proper thresholds
            int redLim = 200;
            int greenLim = 140;
            int blueLim = 50;

            int pixel;

            ArrayList<Integer> left = new ArrayList<>();
            ArrayList<Integer> center = new ArrayList<>();
            ArrayList<Integer> right = new ArrayList<>();

            ArrayList<Integer> range = new ArrayList<>();
            for (int x = 0; x < bm.getWidth(); x++) {
                for (int y = 0; y < bm.getHeight()/2; y++) {
                    pixel = bm.getPixel(x,y);
                    if(red(pixel) > redLim && green(pixel) < greenLim && blue(pixel) < blueLim){
                        range.add(x);
                    }
                }
            }

            int min = Collections.min(range);
            int max = Collections.max(range);

            int leftSum = 0, rightSum = 0, midSum = 0;
            for (Integer p: range) {
                if(p < (min + (max-min)/3)){
                    leftSum++;
                }else if(p < (min + 2*(max-min)/3)){
                    midSum++;
                }else{
                    rightSum++;
                }
                // previously it was adding c, so the column instead of row
                //check once more if 480 is the x and 640 is y
            }

           /* for (int x = 0; x < bm.getWidth()/3; x++) {
                for (int y = 0; y < bm.getHeight(); y++) {
                    left.add(red(bm.getPixel(x,y))); // previously it was adding c, so the column instead of row
                        //check once more if 480 is the x and 640 is y
                }
            }

            for (int x = bm.getWidth()/3; x < 2*bm.getWidth()/3; x++) {
                for (int y = 0; y < bm.getHeight(); y++) {
                    center.add(red(bm.getPixel(x,y))); // previously it was adding c, so the column instead of row
                    //check once more if 480 is the x and 640 is y
                }
            }

            for (int x = 2*bm.getWidth()/3; x < bm.getWidth(); x++) {
                for (int y = 0; y < bm.getHeight(); y++) {
                    right.add(red(bm.getPixel(x,y))); // previously it was adding c, so the column instead of row
                    //check once more if 480 is the x and 640 is y
                }
            }*/

            /*int sum = 0;
            for (Integer x : colorPix)
                sum += x;

            int avgX = 0, maxX = 0, minX =0;
            if(colorPix.size() != 0){
                avgX = sum / colorPix.size();

                maxX = Collections.max(colorPix);
                minX = Collections.min(colorPix);
            }


            if (avgX < 160) {
                stonepos = -1;
            } else if (avgX < 320) {
                stonepos = 0;
            } else {
                stonepos = 1;
            }*/

            ArrayList<Integer> compare = new ArrayList<>();
            compare.add(leftSum);
            compare.add(midSum);
            compare.add(rightSum);

            if(Collections.min(compare) == leftSum)
                stonepos = -1;
            else if (Collections.min(compare) == midSum)
                stonepos = 0;
            else if (Collections.min(compare) == rightSum)
                stonepos = 1;

            telemetry.addData("bitmap width:", bm.getWidth()); //1280
            telemetry.addData("bitmap height:", bm.getHeight()); //720 across I think?);
            telemetry.addData("min:", min);
            telemetry.addData("max:", max);
            telemetry.addData("leftsum: ", leftSum);
            telemetry.addData("midsum: ", midSum);
            telemetry.addData("rightsum: ", rightSum);
            telemetry.addData("stonepos: ", stonepos);
            telemetry.update();
        }else{
            //change it to whatever is closest
            telemetry.addData("Bitmap null:", "Default center(?)");
            telemetry.update();
        }

        //return stonepos;
    }

    public void findThreshold(Bitmap bm) throws InterruptedException {

        if (bm != null) {

            ArrayList<Integer> redPix = new ArrayList<Integer>();
            ArrayList<Integer> greenPix = new ArrayList<Integer>();
            ArrayList<Integer> bluePix = new ArrayList<Integer>();

            for (int c = 0; c < bm.getWidth(); c++) {
                for (int r = 0; r < bm.getHeight(); r++) {
                    redPix.add(red(bm.getPixel(c,r)));
                    greenPix.add(green(bm.getPixel(c,r)));
                    bluePix.add(blue(bm.getPixel(c,r)));
                }
            }

            int redP = 0;
            int greenP = 0;
            int blueP = 0;

            for(Integer p: redPix)
                redP += p;
            for(Integer p: greenPix)
                greenP += p;
            for(Integer p: bluePix)
                blueP += p;

            telemetry.addData("red: ", redP/redPix.size());
            telemetry.addData("green: ", greenP/greenPix.size());
            telemetry.addData("blue: ", blueP/bluePix.size());
            telemetry.update();
        }else{
            //change it to whatever is closest
            telemetry.addData("Bitmap null:", "Default center(?)");
            telemetry.update();
        }
    }

    public void adjustForSkystone(int pos, boolean red) throws InterruptedException{
        if(!red){ // RED
            switch(pos) { // red
                case -1:
                    driveDistance(0.4, 2);
                    break;
                case 0:
                    driveDistance(-0.4, 11);//orig 12
                    break;
                case 1:
                    driveDistance(-0.4, 4);
                    break;
            }
        }else{
            switch(pos) {
                case -1:
                    driveDistance(0.4, 14);
                    break;
                case 0:
                    driveDistance(0.4, 23);
                    break;
                case 1:
                    driveDistance(0.4, 6);
                    break;
            }
        }
        telemetry.update();
    }

    public double forLongAdjust(int pos, boolean red) throws InterruptedException{
        if (!red) { // RED
            switch(pos) {
                case -1:
                    return 15;
                case 0:
                    return 0;
                case 1:
                    return 10;
            }
        } else {
            switch(pos) {
                case -1:
                    return 0;
                case 0:
                    return -10;
                case 1:
                    return 10;
            }
        }
        return 2;//default
    }

    public boolean isRed(){
        ArrayList<Integer> rgb = new ArrayList<>();
        rgb.add(colorSensor.red());
        rgb.add(colorSensor.green());
        rgb.add(colorSensor.blue());

        if(Collections.max(rgb) == colorSensor.red()) {
            return true;
        }else if (Collections.max(rgb) == colorSensor.green()) {
            return true; //figure out red vs green values for red tape
        }else {
            return false;
        }
    }

    public boolean goToLine(double power){
        resetEncoders();
        while (opModeIsActive() && !isStopRequested() && getEncoderAvg() < 0 && !isRed()){ //some value
            setEachMotorPowers(power, power, power, power, false);
        }
        return true;
    }

    //ROBOT ORIENTATION METHODS

    public double getYaw() {
        angles = imu.getAngularOrientation();

        double angle = angles.firstAngle;

        if(angle >= 0){
            return angle;
        }else if (angle < 0){
            return 360 + angle;
        }
        return angle;
    }

    public double get180Yaw() {
        angles = imu.getAngularOrientation();
        return angles.firstAngle;
    }

    public double getResetableYaw(double zeroAngle){
        angles = imu.getAngularOrientation();

        double angle = angles.firstAngle;

        angle -= zeroAngle;

        if (angle < 0){
            angle += 360;
        }

        return angle;
    }

    public double getRobotX() {
        return translation.get(0) / mmPerInch;
    }

    public double getRobotY() {
        return translation.get(1) / mmPerInch;
    }

    public double getRobotZ() {
        return translation.get(2) / mmPerInch;
    }

    public double getRobotHeading(){
        return rotation.thirdAngle;
    }

    public void position(){ //constantly gives X and Y position of robot
        while(!isStopRequested()){
            telemetry.addData("X -", getRobotX());
            telemetry.addData("Y -", getRobotY());
            telemetry.update();
        }
    }

    public boolean isRed(int timeout) { // in milliseconds
        runtime.reset();
        CameraDevice camera = CameraDevice.getInstance();
        camera.setFlashTorchMode(true);

        while (opModeIsActive() && !isStopRequested() && runtime.milliseconds() < timeout) {
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                        translation = lastLocation.getTranslation();
                    }
                    camera.setFlashTorchMode(false);
                    switch (trackable.getName()) {
                        default:
                            telemetry.addData("No Trackable", "Detected");
                        case "Red Perimeter 2":
                            return true;
                        case "Red Perimeter 1":
                            return true;
                    }

                }
            }
        }
        return false;
    }

    public void disableTracking(){
        targetsSkyStone.deactivate();
        telemetry.addData("Tracking: ", "Disabled");
        telemetry.update();
    }

    public void activateTfod(){
        if (tfod != null)
            tfod.activate();
    }

    public void deactivateTfod(){
        tfod.deactivate(); //is both deactivate and shutdown necessary?
        tfod.shutdown();
    }

    /*public void getDistance(){
        telemetry.addData("distance from sensor: ", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }*/

    //UNUSED METHODS

    //public Bitmap convertToBitmap() throws InterruptedException{
    /**    Image rgb = null;
     com.vuforiaPC.Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
     Bitmap imageBitmap;
     vuforiaPC.setFrameQueueCapacity(1);
     vuforiaPC.enableConvertFrameToBitmap();
     VuforiaLocalizer.CloseableFrame picture;

     frame = vuforiaPC.getFrameQueue();
     picture = frame.take();

     long imgCount = picture.getNumImages(); //GET NUMBER OF FORMATS FOR FRAME

     for (int i = 0; i < imgCount; i++){
     if(picture.getImage(i).getFormat() == PIXEL_FORMAT.RGB565){
     rgb = picture.getImage(i);
     break;
     }
     }

     imageBitmap = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565 );
     imageBitmap.copyPixelsFromBuffer(rgb.getPixels());

     picture.close();
     return imageBitmap;
     }**/

    //public void detectSkystone(double timeLimit){
    /** runtime.reset();
     activateTfod();
     while(runtime.seconds() < timeLimit && opModeIsActive() && tfod != null) {
     // getUpdatedRecognitions() will return null if no new information is available since the last time that call was made.
     List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
     if (updatedRecognitions != null) {
     telemetry.addData("# Object Detected", updatedRecognitions.size());
     if (updatedRecognitions.size() > 0) { //IF DETECT BOTH OBJECTS

     int skystoneX = -1, stone1X = -1, stone2X = -1;
     double skystoneConf = 0;

     for (Recognition recognition : updatedRecognitions) {
     if (recognition.getLabel().equals(LABEL_SKYSTONE)) { //IF OBJECT DETECTED IS GOLD
     skystoneX = (int) recognition.getLeft();
     skystoneConf = recognition.getConfidence();
     } else if (recognition.getLabel().equals(LABEL_STONE) && stone1X != -1) {
     stone1X = (int) recognition.getLeft();
     } else {
     stone2X = (int) recognition.getLeft();
     }
     }

     //Adjust based on if it can see 3 stones or 2 stones
     //Ask galligher how to get the y-value of the object, but could also use
     //ratio of block height to frame height

     //(*^*)\\ <-- A CHICK!

     if (skystoneX != -1 && skystoneConf > 0.2) { //adjust confidence level

     if (skystoneX < 600 || (skystoneX < stone1X && skystoneX < stone2X)) { //adjust threshold3
     telemetry.addData("Skystone Position", "Left");
     pos = 1;
     } else{
     telemetry.addData("Skystone Position", "Center");
     pos = 2;
     }
     }else{
     telemetry.addData("Skystone Position", "Right");
     pos = 3;
     }
     telemetry.addData("Gold x pos ", skystoneX);
     telemetry.addData("Gold conf ", skystoneConf);
     telemetry.addData("Runtime", getTime());
     telemetry.update();
     }
     }
     }
     }
     **/

    //public int detectSkystone(){
    /**   //USE VUFORIA TRACKABLE COORDINATE TO DETERMINE SKYSTONE POSITION
     //GO GET COORDINATES OF THE TARGET IN EACH POSITION
     OpenGLMatrix skystonePos = null;
     VectorF skystoneCoords = null;
     ElapsedTime runtime = new ElapsedTime();
     runtime.reset();
     int pos = 0;
     while(opModeIsActive()) {
     for (VuforiaTrackable trackable : allTrackables) {
     if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
     telemetry.addData("Visible Target", trackable.getName());

     if (trackable.getName() == "Stone Target") {
     skystonePos = trackable.getLocation();
     skystoneCoords = skystonePos.getTranslation();

     if (skystoneCoords.get(0) / mmPerInch < 0 && skystoneCoords.get(0) / mmPerInch > 0) {
     pos = -1;
     } else if (skystoneCoords.get(0) / mmPerInch < 0 && skystoneCoords.get(0) / mmPerInch > 0) {
     pos = 0;
     } else if (skystoneCoords.get(0) / mmPerInch < 0 && skystoneCoords.get(0) / mmPerInch > 0) {
     pos = 1;
     } else {
     pos = 0;
     }
     }
     break;
     }
     }
     }
     telemetry.addData("Skystone Pos:", pos);
     telemetry.update();
     return pos;
     }**/

    //public void turnPID(double tAngle, double kP, double kI, double kD, double timeOut){
    /** double power, prevError, error, dT, prevTime, currTime, P, I, D; //DECLARE ALL VARIABLES
     prevError = error = tAngle - getYaw(); //INITIALIZE THESE VARIABLES
     power = dT = prevTime = currTime = P = I = D = 0;
     ElapsedTime time = new ElapsedTime(); //CREATE NEW TIME OBJECT
     resetTime();
     while (Math.abs(error) > 0.5 && currTime < timeOut){
     prevError = error;
     error = tAngle - getYaw(); //GET ANGLE REMAINING TO TURN (tANGLE MEANS TARGET ANGLE, AS IN THE ANGLE YOU WANNA GO TO)
     prevTime = currTime;
     currTime = time.milliseconds();
     dT = currTime - prevTime; //GET DIFFERENCE IN CURRENT TIME FROM PREVIOUS TIME
     P = error;
     I = error * dT;
     D = (error - prevError)/dT;
     power = P * kP + I * kI + D * kD;
     setMotorPowers(Range.clip(power, 0.2, 1), -Range.clip(power, 0.2, 1));

     telemetry.addData("tAngle: ", tAngle)
     .addData("P:", P)
     .addData("I:", I)
     .addData("D:", D)
     .addData("power", power)
     .addData("error: ", error)
     .addData("currTime: ", currTime);
     }
     }
     **/

    //public void colorPark(boolean red){
    /** Color.RGBToHSV(sensorColor.red() * 8, sensorColor.green() * 8, sensorColor.blue() * 8, hsvValues);
     float hue = hsvValues[0];
     boolean redPark = hue < 60 || hue > 320; //red hues
     boolean bluePark = hue > 120 && hue < 260; /blue hues

     if(red){
     while(!redPark && !isStopRequested()){
     setMotorPowers(0.5, 0.5);
     }
     stopMotors();
     }
     else{
     while(!bluePark && !isStopRequested()){
     setMotorPowers(0.5, 0.5);
     }
     stopMotors();
     }
     **/

    //public void strafeDistance(double power, boolean right, double dist) {  // Garrett(10/22/19)
    /**    //Declare variables
     double min = 0.3;   //adjustable minimum power for strafing
     double powerG = power * 0.4;   //PowerGiven = Starts out with a lower power so the robot doesn't drift as much
     dist *= -1 * encoderToInches;   //sets distance (will change after testing to see how short it is of target distance)
     LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    //resets motor encoders
     RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //set the mode of the motors
     RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

     if (right) {  //If strafing right
     //Sets the target position for the motors to move to
     LF.setTargetPosition(LF.getCurrentPosition() + (int)dist);
     LB.setTargetPosition(LB.getCurrentPosition() - (int)dist);
     RF.setTargetPosition(RF.getCurrentPosition() - (int)dist);
     RB.setTargetPosition(RB.getCurrentPosition() + (int)dist);
     //tells motors to move to positions set
     LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     //sets motor powers
     LF.setPower(-Range.clip(powerG, min, 1));
     RF.setPower(Range.clip(-powerG, min, 1));
     LB.setPower(Range.clip(-powerG, min, 1));
     RB.setPower(-Range.clip(powerG, min, 1));
     }
     else {    //If strafing left
     //Sets the target position for the motors to move to
     LF.setTargetPosition(LF.getCurrentPosition() - (int)dist);
     LB.setTargetPosition(LB.getCurrentPosition() + (int)dist);
     RF.setTargetPosition(RF.getCurrentPosition() + (int)dist);
     RB.setTargetPosition(RB.getCurrentPosition() - (int)dist);
     //tells motors to move to positions set
     LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     //sets motor powers
     LF.setPower(Range.clip(-powerG, min, 1));
     RF.setPower(-Range.clip(powerG, min, 1));
     LB.setPower(-Range.clip(powerG, min, 1));
     RB.setPower(Range.clip(-powerG, min, 1));
     }

     //Waits until all the motors reach their target position then stops them.
     //Also incrementally increases speed of the robot to full desired power
     while (LF.isBusy() && RF.isBusy() && LB.isBusy() && RB.isBusy() && opModeIsActive() && !isStopRequested()){
     if(right) {
     LF.setPower(-Range.clip(powerG, min, 1));
     RF.setPower(Range.clip(-powerG, min, 1));
     LB.setPower(Range.clip(-powerG, min, 1));
     RB.setPower(-Range.clip(powerG, min, 1));
     }
     if(right == false) {
     LF.setPower(-Range.clip(powerG, min, 1));
     RF.setPower(Range.clip(-powerG, min, 1));
     LB.setPower(Range.clip(-powerG, min, 1));
     RB.setPower(-Range.clip(powerG, min, 1));
     }
     //If statement makes sure that powerG stops increasing once it is equal with power
     if (powerG < power) {
     powerG += powerG * 0.1;
     }
     }
     stopMotors();
     }*/

    //public void strafeAdjust(double power, double distance, boolean right, int timeout){
    /**
     double deltaHeading = 0;

     resetEncoders();
     while (getEncoderAvg() < distance * 55 && !isStopRequested()) {
     deltaHeading = getYaw();
     power = Range.clip(deltaHeading/0.5, 0.25, 1);

     if (Math.abs(getYaw()-90) > 5){
     driveTime(1,0.5);
     }else{
     setStrafePowers(power,right);
     }

     }
     }**/
    //Hello
    //asha
    //Hello world
    //This is Taran
    @Override
    public void runOpMode() throws InterruptedException {

    }
}