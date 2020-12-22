package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Arrays;

import static android.graphics.Color.red;

public class UltimateGoalLinearOpMode extends LinearOpMode {

    // DECLARE VARIABLES TO BE USED
    ElapsedTime runtime;

    //MOTORS
    public DcMotor LF;
    public DcMotor RF;
    public DcMotor LB;
    public DcMotor RB;
    public DcMotor intake;
    public DcMotor shooter;
    public Servo loader;

    //SENSORS
    public BNO055IMU imu;

    //GYRO VARIABLES
    Orientation angles;

    //ticks per inch = (Motor revolutions * gear up ratio) / (wheel diameter * pie)
    //Motor revolutions = COUNTS_PER_MOTOR_REV
    //gear up ratio = 2:1   (ratio beyond motor)
    //wheel diameter = WHEEL_DIAMETER_INCHES
    static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // REV Motor Encoder (1120 for 40:1) (560 for 20:1) (336 for 12:1)
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;   // This is < 1.0 if geared UP   (ratio is 2:1)
    static final double     WHEEL_DIAMETER_INCHES   = 4;     // For figuring circumference

    public double encoderToInches = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * Math.PI);
    public double strafeEncoderToInches = 60;

    // VUFORIA VARIABLES
    public static final String VUFORIA_KEY = "AQt2xVL/////AAABmXIVKUnTcEJbqvVBjp/Sw/9SqarohYyKotzRjT/Xl1/S8KDwsFHv/zYw6rXqXTjKrnjk92GfBA4hbZaQP17d1N6BiBuXO2W/hFNoMGxiF+fWlnvtDmUM1H/MF9faMOjZcPNjnQ7X8DVwdDDha3A3aqaoegefkKxb4A5EjP8Xcb0EPJ1JA4RwhUOutLbCDJNKUq6nCi+cvPqShvlYTvXoROcOGWSIrPxMEiOHemCyuny7tJHUyEg2FTd2upiQygKAeD+LN3P3cT02aK6AJbQ0DlQccxAtoo1+b//H6/eGro2s0fjxA2dH3AaoHB7qkb2K0Vl7ReFEwX7wmqJleamNUG+OZu7K3Zm68mPudzNuhAWQ";
    private VuforiaLocalizer vuforia;

    private VuforiaLocalizer.CloseableFrame frame; //takes the frame at the head of the queue
    private Image rgb;
    private Image greyscale;

    // INITIALIZE
    public void init(HardwareMap map, boolean auto){

        runtime     = new ElapsedTime();
        LF          = map.dcMotor.get("LF");
        RF          = map.dcMotor.get("RF");
        LB          = map.dcMotor.get("LB");
        RB          = map.dcMotor.get("RB");
        intake      = map.dcMotor.get("intake");
        shooter     = map.dcMotor.get("shooter");
        loader      = map.servo.get("loader");
        imu         = map.get(BNO055IMU.class, "imu"); // Check which IMU is being used

        LF.setDirection(DcMotorSimple.Direction.REVERSE);//goes backward on positive speed so reverse needed
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);//goes backward on positive speed so reverse needed
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //SET UP GYRO
        angles = new Orientation();

        resetEncoders();

        // Make sure loader is retracted
        setLoader(false);

        if (auto) {
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

            /*initVuforia();
            telemetry.addData("Vuforia status:", "Initialized");
            telemetry.update();*/
        }

        // FINAL MESSAGE
        telemetry.addData("Status: ", "All Initialized");
        telemetry.update();
    }

    // INIT BITMAP VUFORIA
    public void initBitmapVuforia(){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time

        telemetry.addData("Vuforia:", "initialized");
        telemetry.update();
    }

    // GET BITMAP
    public Bitmap getBitmap() throws InterruptedException {
        Bitmap bm = null;

        if(opModeIsActive()&& !isStopRequested()){
            frame = vuforia.getFrameQueue().take();
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

    /*
    public Bitmap getBitmapGrayScale() throws InterruptedException {
        Bitmap bm = null;

        if(opModeIsActive()&& !isStopRequested()){
            frame = vuforia.getFrameQueue().take();
            long num = frame.getNumImages();

            for(int i = 0; i < num; i++){
                if(frame.getImage(i).getFormat() == PIXEL_FORMAT.GRAYSCALE){
                    greyscale = frame.getImage(i);
                }
            }

            bm = Bitmap.createBitmap(greyscale.getWidth(), greyscale.getHeight(), Bitmap.Config.ARGB_8888);
            bm.copyPixelsFromBuffer(greyscale.getPixels());
        }

        frame.close();
        return bm;
    }
    */

    // GET NUMBER OF RINGS IN STACK - might have to calibrate at competition field bc of lighting
    // ***WE CAN ALSO ATTACH A COLOR/LIGHT SENSOR TO GIVE LIGHT
    // NOTE: Need to drive up to first mat line (the end of the tape)
    public int detectStack(Bitmap bm, boolean left){

        // frame height = 720 px, frame width = 1280

        int numRings = 1; // pick closest for default value
        int ringRed = 130; // current compromise in terms of lighting differences
        int totalRed = 0;

        if (left){
            for (int x = 600; x < 800; x++){
                for (int y = 360; y < bm.getHeight(); y++ ){
                    if (red(bm.getPixel(x,y)) > ringRed)
                        totalRed++;
                }
            }
        }else{
            // need to do right side
        }

        long one = 8000;
        long two = 10000;
        long three = 12000;

        if (totalRed < one){
            numRings = 1;
        } else if (totalRed > two && totalRed < three){
            numRings = 2;
        } else if (totalRed > three){
            numRings = 3;
        }

        // cut off top and bottom section of frame
        // get total number of yellow pixels
        // the greater the number of red pixels, the more number of rings

        if (bm != null) {
            telemetry.addData("numRings: ", numRings);
            telemetry.addData("totalRed: ", totalRed);
            telemetry.update();
            sleep(1000);
        }else{
            telemetry.addData("Bitmap null:", "Default 1");
            telemetry.update();
        }

        return numRings;
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
        /*int avg = (Math.abs(LF.getCurrentPosition())
                + Math.abs(LB.getCurrentPosition())
                + Math.abs(RB.getCurrentPosition())
                + Math.abs(RF.getCurrentPosition()))/4;*/

        int avg = (Math.abs(LF.getCurrentPosition())
                + Math.abs(RF.getCurrentPosition()))/2;

        // currently using front two motors because they have similar and accurate encoder values
        // the back two motors are off - one is a bit over, one is a lot under

        return avg;
    }

    //SET EACH MOTOR POWERS
    public void stopMotors() {
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
    }

    public void setMotorPowersCorrection(String type, double lf_L, double rf_R, double lb, double rb) {
        double[] powers = {lf_L,rf_R,lb,rb};
        double[] corrections = {4696.0/4902,4696.0/4895,4696.0/5091,1};
        //double[] corrections = {1.0,1.0,1.0,1.0};
        for (int i = 0; i < 4; i++){
             powers[i] = Range.clip(powers[i], -1, 1);
        }

        switch (type) {
            case "ALL":
                LF.setPower(powers[0]*corrections[0]);
                RF.setPower(powers[0]*corrections[1]);
                LB.setPower(powers[0]*corrections[2]);
                RB.setPower(powers[0]*corrections[3]);
                break;
            case "SIDES":
                LF.setPower(powers[0]*corrections[0]);
                RF.setPower(powers[1]*corrections[1]);
                LB.setPower(powers[0]*corrections[2]);
                RB.setPower(powers[1]*corrections[3]);
                break;
            case "EACH":
                LF.setPower(powers[0]*corrections[0]);
                RF.setPower(powers[1]*corrections[1]);
                LB.setPower(powers[2]*corrections[2]);
                RB.setPower(powers[3]*corrections[3]);
                break;
            default:
                telemetry.addData("Error:", "motor powers set incorrectly");
                telemetry.update();
        }
        telemetry.addData("corrections", Arrays.toString(powers));
        telemetry.update();
    }

    // SET MOTOR POWERS IN DIFFERENT FORMATS (CHECK DOCUMENTATION FOR EACH CASE)
    public void setMotorPowers(String type, double lf_L, double rf_R, double lb, double rb) {
        double[] powers = {lf_L,rf_R,lb,rb};

        for (int i = 0; i < 4; i++){
            if (powers[i] != 0)
                powers[i] = (powers[i] > 0) ? Range.clip(powers[i],0.3,1) : Range.clip(powers[i],-1,-0.3);
            // figure out 0.3 or 0.2 minimum and retest PID constants
        }

        switch (type) {

            // ALL - all motor powers will be set to the value in the first double parameter
            case "ALL":
                LF.setPower(powers[0]);
                RF.setPower(powers[0]);
                LB.setPower(powers[0]);
                RB.setPower(powers[0]);
                break;

            // SIDES - the left motors will be set to the powers in the first double parameter
            // and the right motors will be set to the powers in the second double parameter
            case "SIDES":
                LF.setPower(powers[0]);
                RF.setPower(powers[1]);
                LB.setPower(powers[0]);
                RB.setPower(powers[1]);
                break;

            // EACH - each motor will be set to the powers in the corresponding parameters in the order lf-rf-lb-rb
            case "EACH":
                LF.setPower(powers[0]);
                RF.setPower(powers[1]);
                LB.setPower(powers[2]);
                RB.setPower(powers[3]);
                break;

            // STRAFE - all motors will strafe at the power given in the first double parameter
            // *** If you want to strafe right, give + power (- power for left)
            case "STRAFE":
                LF.setPower(powers[0]);
                RF.setPower(-powers[0]);
                LB.setPower(-powers[0]);
                RB.setPower(powers[0]);
                break;

            // STRAFE ADJUST - left rotating forces will be set to power given in first double parameter
            // right rotating forces will be set to power given in second double parameter
            case "STRAFE ADJUST":
                LF.setPower(powers[1]);
                RF.setPower(-powers[1]);
                LB.setPower(-powers[0]);
                RB.setPower(powers[0]);
                break;

            default:
                telemetry.addData("Error:", "motor powers set incorrectly");
        }
        telemetry.update();
    }

    public void setMotorPowersDynamicCorrection(String type, double lf_L, double rf_R, double lb, double rb, double[] corrections) {
        double[] powers = {lf_L,rf_R,lb,rb};

        for (int i = 0; i < 4; i++){
            powers[i] = Range.clip(powers[i], -1, 1);
        }

        switch (type) {
            case "ALL":
                LF.setPower(powers[0]*corrections[0]);
                RF.setPower(powers[0]*corrections[1]);
                LB.setPower(powers[0]*corrections[2]);
                RB.setPower(powers[0]);
                break;
            case "SIDES":
                LF.setPower(powers[0]*corrections[0]);
                RF.setPower(powers[1]*corrections[1]);
                LB.setPower(powers[0]*corrections[2]);
                RB.setPower(powers[1]);
                break;
            case "EACH":
                LF.setPower(powers[0]*corrections[0]);
                RF.setPower(powers[1]*corrections[1]);
                LB.setPower(powers[2]*corrections[2]);
                RB.setPower(powers[3]);
                break;
            default:
                telemetry.addData("Error:", "motor powers set incorrectly");
                telemetry.update();
        }
    }

    // SHOW MOTOR TELEMETRY DATA
    public void motorTelemetry(){
        telemetry.addData("avg:", getEncoderAvg());
        telemetry.addData("LF Power", LF.getPower() + " " + LF.getCurrentPosition());
        telemetry.addData("RF Power", RF.getPower() + " " + RF.getCurrentPosition());
        telemetry.addData("LB Power", LB.getPower() + " " + LB.getCurrentPosition());
        telemetry.addData("RB Power", RB.getPower() + " " + RB.getCurrentPosition());
    }

    public void testEncoders(int targetTicks, double pwr){
        resetEncoders();
        while (opModeIsActive() && !isStopRequested() && getEncoderAvg() < targetTicks){

            if (RB.getCurrentPosition() == 0){
                setMotorPowers("ALL", pwr,0,0,0);
            }else {
                double[] corrections = {
                        RB.getCurrentPosition()/LF.getCurrentPosition(),
                        RB.getCurrentPosition()/RF.getCurrentPosition(),
                        RB.getCurrentPosition()/LB.getCurrentPosition()};
                telemetry.addData("corrections", Arrays.toString(corrections));
                setMotorPowersDynamicCorrection("ALL", pwr, 0, 0, 0, corrections);
            }
            motorTelemetry();
            telemetry.update();
        }
        stopMotors();
    }

    // STRAFE DISTANCE
    public void strafeDistance(double power, double inches, double seconds){

        // positive power is right strafe and negative power is left strafe
        double total = inches * strafeEncoderToInches;
        ElapsedTime t = new ElapsedTime();
        t.reset();
        resetEncoders();

        while (opModeIsActive() && !isStopRequested() && getEncoderAvg() < total && t.seconds() < seconds){

            setMotorPowers("STRAFE",power,0,0,0);

            telemetry.addData("target:", total);
            motorTelemetry();
            telemetry.update();
        }
        stopMotors();
    }

    // STRAFE ADJUST W/ GYRO ADJUSTMENT
    public void strafeAdjust(double power, double inches, double tHeading, double seconds){
        // ORIENTATION -180 TO 180
        // LEFT = +, RIGHT = -

        double total = (inches) * strafeEncoderToInches; // -2 to account for drift
        double remaining, finalPower, error, rp, lp, p = 1.2;
        ElapsedTime t = new ElapsedTime();
        t.reset();
        resetEncoders();

        while (opModeIsActive() && !isStopRequested() && getEncoderAvg() < total && t.seconds() < seconds) {
            remaining = total - getEncoderAvg();
            error = tHeading - get180Yaw(); //GET ANGLE REMAINING TO TURN (tANGLE MEANS TARGET ANGLE, AS IN THE ANGLE YOU WANNA GO TO)

            if(Math.abs(error) > 180) error = (error < 0) ? 360 + error : 360 - error;

            finalPower = (remaining / total) * power;
            if (finalPower != 0) finalPower = (finalPower > 0) ? Range.clip(finalPower,0.3,1) : Range.clip(finalPower,-1,-0.3);

            rp = finalPower;
            lp = finalPower;

            // when moving right: right rotating forces are LF and RF, left rotating forces are LB and RB
            // when moving left: right rotating forces are LB and RB, left rotating forces are LF and RF

            // positive error = need to turn right
            // negative error = need to turn left

            // note: left strafe is worse than right strafe and may not always reach full distance

            if (power != 0){
                if (power < 0) {
                    if (error > 1)
                        rp *= p;
                    else
                        lp *= p;
                } else {
                    if (error < -1) // why is it the same changes as when error > 1 ??? hmmmm
                        rp *= p;
                    else
                        lp *= p;
                }
            }

            setMotorPowers("STRAFE ADJUST", lp, rp,0,0);

            telemetry.addData("target:", total);
            telemetry.addData("error", error);
            motorTelemetry();
            telemetry.update();
        }
        stopMotors();
    }

    // DRIVE FORWARD & BACKWARD
    public void driveDistance(double maxPower, double inches, double seconds){
        double total = inches * encoderToInches;
        double remaining, finalPower;
        ElapsedTime t = new ElapsedTime();
        t.reset();
        resetEncoders();
        while (opModeIsActive() && !isStopRequested() && getEncoderAvg() < inches * encoderToInches && t.seconds() < seconds){
            remaining = total - getEncoderAvg();
            telemetry.addData("remaining:", remaining);
            finalPower = (remaining/total) * maxPower;
            setMotorPowers("ALL",finalPower,0,0,0);
            telemetry.addData("target:", inches*encoderToInches);
            //motorTelemetry();
        }
        setMotorPowers("ALL", 0,0,0,0);
    }

    // DRIVE W/ GYRO ADJUSTMENT
    public void driveAdjust(double tHeading, double power, double inches, double seconds){
        // ORIENTATION -180 TO 180
        // LEFT = +, RIGHT = -

        double total = (inches) * encoderToInches; // -2 to account for drift
        double remaining, finalPower, error, lp, rp, m = 1.5;
        ElapsedTime t = new ElapsedTime();
        t.reset();
        resetEncoders();

        while (opModeIsActive() && !isStopRequested() && getEncoderAvg() < total && t.seconds() < seconds) {
            remaining = total - getEncoderAvg();
            error = tHeading - get180Yaw(); //GET ANGLE REMAINING TO TURN (tANGLE MEANS TARGET ANGLE, AS IN THE ANGLE YOU WANNA GO TO)

            if(Math.abs(error) > 180) error = (error < 0) ? 360 + error : 360 - error;

            finalPower = (remaining / total) * power;
            if (finalPower != 0) finalPower = (finalPower > 0) ? Range.clip(finalPower,0.3,1) : Range.clip(finalPower,-1,-0.3);

            rp = finalPower;
            lp = finalPower;

            // unfortunately there is about an inch of lateral deviation

            if (power < 0) {
                if (error > 1){
                    lp *= 1.25;
                } else if (error < -1)
                    rp *= 1.75;
            } else {
                if (error < 1) {
                    rp *= m;
                }else if (error < -1)
                    lp *= m;
            }

            setMotorPowers("SIDES",lp,rp,0,0);

            telemetry.addData("left power: ", lp)
                    .addData("right power: ", rp)
                    .addData("error", error)
                    .addData("current angle", get180Yaw())
                    .addData("target angle", tHeading);
            telemetry.update();
        }
        stopMotors();
    }

    // PID TURNING
    public void turnPID(double tAngle, double P, double I, double D, double seconds){
        // ORIENTATION -180 TO 180
        // - is right, + is left
        // constants P: 0.8/180    I: 0.0001   D: 0.5 <--last years

        //turnPID(90,0.8/180,0.0001,0.5,5000);
        //turnPID(180,0.8/180,0.00005,0.1,5000);

        double power, prevError, error, dT, prevTime, currTime; //DECLARE ALL VARIABLES

        double kP = P;
        double kI = I;
        double kD = D;

        //prevError =
        error = tAngle - get180Yaw(); //INITIALIZE THESE VARIABLES

        currTime = 0.0;

        ElapsedTime t = new ElapsedTime(); //CREATE NEW TIME OBJECT
        t.reset();
        while (opModeIsActive() && Math.abs(error) > 0.7 && currTime < seconds){
            prevError = error;
            error = tAngle - get180Yaw(); //GET ANGLE REMAINING TO TURN (tANGLE MEANS TARGET ANGLE, AS IN THE ANGLE YOU WANNA GO TO)

            if(error > 180){
                error = -(error-180);
            }else  if(error < -180){
                error = -(error+180);
            }

            prevTime = currTime;
            currTime = t.milliseconds();
            dT = currTime - prevTime; //GET DIFFERENCE IN CURRENT TIME FROM PREVIOUS TIME
            power = (error * kP) + ((error) * dT * kI) + ((error - prevError)/dT * kD);

            if (power < 0)
                //setMotorPowers("SIDES",Range.clip(-power, 0.2, 0.5), Range.clip(power, -0.5, -0.2),0,0);
                setMotorPowers("SIDES",-power, power,0,0);
            else
                //setMotorPowers("SIDES",Range.clip(-power, -0.5, -0.2), Range.clip(power, 0.2, 0.5),0,0);
                setMotorPowers("SIDES",-power, power,0,0);

            telemetry.addData("tAngle: ", tAngle)
                    .addData("currAngle: ", get180Yaw())
                    .addData("kP:", error * kP)
                    .addData("kI:", error * dT * kI)
                    .addData("kD:", (error - prevError)/dT * kD)
                    .addData("power", power)
                    .addData("error: ", error)
                    .addData("currTime: ", currTime);
            motorTelemetry();
            telemetry.update();
        }
        stopMotors();
    }

    // GET IMU VALUE ON -180 TO 180 SCALE
    public double get180Yaw() { return imu.getAngularOrientation().firstAngle; }

    // INTAKE RINGS
    public void runIntake(long milliseconds){
        // Ask mr galligher if i have to increment this as well like the flywheel
        intake.setPower(-0.8);
        sleep(milliseconds);
        intake.setPower(0);
    }

    // START INTAKE MOTOR
    public void startIntake(){
        // Ask mr galligher if i have to increment this as well like the flywheel
        intake.setPower(-0.8);
    }

    //LOAD RINGS
    public void setLoader(boolean deploy){
        if (deploy)
            loader.setPosition(0);
        else
            loader.setPosition(1);

        sleep(1500);
        // reduce or remove wait time if needed
    }

    // SHOOT RINGS (DO PHYSICS CALCULATIONS)
    public void runShooter(long milliseconds){
        // ask Mr. Galligher if incrementing intake speed in tele-op is necessary
        // TIP: the brake mode should be a float for the fly wheel, increment/step up to full power for flywheel (within like half a second)

        double speed = 0;

        // adjust max speed as needed
        while (speed < 1) {
            speed += 0.2;
            shooter.setPower(speed);
            telemetry.addData("shooter speed:", speed);
            telemetry.update();
            sleep(100);
        }

        sleep(milliseconds);
        shooter.setPower(0);
    }

    // INCREMENTALLY STARTS SHOOTER
    public void startShooter(){
        // ask Mr. Galligher if incrementing intake speed in tele-op is necessary
        // TIP: the brake mode should be a float for the fly wheel, increment/step up to full power for flywheel (within like half a second)

        double speed = 0;
        double max = 0.75;

        // adjust max speed as needed
        while (speed < max) {
            speed += max/5;
            shooter.setPower(speed);
            telemetry.addData("shooter speed:", speed);
            telemetry.update();
            sleep(100);
        }
    }

    // FULL INTAKE AND SHOOTING PROCESS - ENTER NUMBER OF RINGS TO SHOOT
    public void launchCycle(int numCycles){
        // roughly written, still need to figure out timing between intake and loader
        startShooter();
        startIntake();
        for (int i = 0; i < numCycles; i++){
            sleep(1000);
            setLoader(true);
            telemetry.addData("Shot:", i+1);
            telemetry.update();
            setLoader(false);
            telemetry.addData("Load next", "");
            telemetry.update();
        }
        shooter.setPower(0);
        intake.setPower(0);
    }

    //GRAB & RELEASE WOBBLE

    //******************************************************************************//

    // AUTONOMOUS METHODS:

    // ONE METHOD FOR MOVING BACK AND FORTH BETWEEN DROP ZONES AND LAUNCH LINE (PARAMS: WHICH BOX TO GO TO, WHICH LINE START AT)

    // ONE METHOD TO DETECT THE NUMBER RINGS STACKED IN AUTO (PARAMS: BITMAP OBJECT)

    // ONE METHOD TO DO POWERSHOTS IN AUTO (PARAMS: BOX)

    // ONE METHOD WITH COLOR SENSOR TO LINE UP WITH LAUNCH LINE

    // ONE METHOD TO SET ___

    @Override
    public void runOpMode() throws InterruptedException {}
}