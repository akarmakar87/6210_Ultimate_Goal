package org.firstinspires.ftc.teamcode.X_Other;


import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static android.graphics.Color.red;

/**
 * PURPOSE: This class is made to store all the methods from UltimateGoalLinearOpMode that we are no longer using,
 * so that we can keep that class neat without having to delete old methods.
 */

public class DeprecatedMethods extends LinearOpMode {

    //-----VUFORIA VARIABLES-----//

    public static final String VUFORIA_KEY = "AQt2xVL/////AAABmXIVKUnTcEJbqvVBjp/Sw/9SqarohYyKotzRjT/Xl1/S8KDwsFHv/zYw6rXqXTjKrnjk92GfBA4hbZaQP17d1N6BiBuXO2W/hFNoMGxiF+fWlnvtDmUM1H/MF9faMOjZcPNjnQ7X8DVwdDDha3A3aqaoegefkKxb4A5EjP8Xcb0EPJ1JA4RwhUOutLbCDJNKUq6nCi+cvPqShvlYTvXoROcOGWSIrPxMEiOHemCyuny7tJHUyEg2FTd2upiQygKAeD+LN3P3cT02aK6AJbQ0DlQccxAtoo1+b//H6/eGro2s0fjxA2dH3AaoHB7qkb2K0Vl7ReFEwX7wmqJleamNUG+OZu7K3Zm68mPudzNuhAWQ";
    private VuforiaLocalizer vuforia;
    private VuforiaLocalizer.CloseableFrame frame; // Takes the frame at the head of the queue
    private Image rgb;

    /**
     * PURPOSE: Get the Bitmap version of the image from the phone camera
     * NOTE: A Bitmap is basically an array of pixels, each of which has an RGB value that you can analyze
     * @return the Bitmap of image frame taken at that instant
     * @throws InterruptedException
     */
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

    /**
     * PURPOSE: Get # of rings in stack
     * NOTE: In order to detect properly, robot must drive up to first mat line (the end of the tape)
     * ISSUES: Lighting causes red values to vary drastically - we could also attach a color/light sensor to give light
     * @param: bm - always write "getBitmap()" in this parameter
     *           NOTE: You are basically passing in the Bitmap object returned from the getBitmap() method
     * @param: left - no matter the alliance color, if the robot is on the left of the stack, write True - otherwise write False
     * @return the # of rings detected
     */
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

    /**
     * PURPOSE: Initialize the Vuforia instance for bitmapping
     * NOTE: Basically, it turns on the phone camera - if you don't need the camera at the moment, comment out this method
     */
    public void initBitmapVuforia() {

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

    /****MANIPULATOR MECHANISM METHODS****/
    /**
     * PURPOSE: Complete full launch cycles
     * ISSUE: Roughly written, might need to tweak timing between intake and loader
     * @param zone - I just put this as an idea that you could specify the zone you want to shoot in and it will set the speed accordingly
     * @param numCycles - enter the # of rings you want to launch
     */
    public void launchCycle(int zone, int numCycles){

        switch (zone){
            case 1:
            case 2:
            case 3:
                //startShooter(0.75); // These are placeholders for the actual speed values
                break;
            default:
                telemetry.addData("Error:","Zone not specified correctly");
                telemetry.update();
        }
        //startIntake();

        for (int i = 0; i < numCycles; i++){
            sleep(1000);
           // setLoader(true);
            telemetry.addData("Shot:", i+1);
            telemetry.update();
           // setLoader(false);
            telemetry.addData("Load next", "");
            telemetry.update();
        }
        //shooter.setPower(0);
       // intake.setPower(0);
    }

    /**
     * PURPOSE: Run the intake for a certain amount of time
     * NOTE: Power is currently set to 0.8 because anything greater launches the wheel over the loading zone
     * ISSUES: Not an issue, but I'm not sure if I'm supposed to increment the starting speed like I did for the flywheel...
     * @param milliseconds - enter the number of milliseconds you want intake to run for (1000 milliseconds = 1 second)
     */
    public void runIntake(long milliseconds){
        //intake.setPower(-0.8);
        sleep(milliseconds);
        //intake.setPower(0);
    }

    /**
     * PURPOSE: Starts intake motor - that's it
     */
    public void startIntake(){
        //intake.setPower(-0.8);
    }

    @Override
    public void runOpMode() throws InterruptedException {}
}