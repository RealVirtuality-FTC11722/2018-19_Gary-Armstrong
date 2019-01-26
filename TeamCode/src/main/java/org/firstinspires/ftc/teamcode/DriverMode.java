package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Lyesome on 2018-01-13.
 * This file contains all the instructions for controlling the robot in Teleop mode.
 */

@TeleOp(name="Driver Mode - Only", group="Linear OpMode")  // @Autonomous(...) is the other common choice
//@Disabled
public class DriverMode extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    //create a new robot named astroGary
    private BotConfig astroGary = new BotConfig();


    @Override
    public void runOpMode() {

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        //Provide warning for drivers not to hit play until initializing is complete.
        telemetry.addData("Status", "Initializing. Please Wait...");
        telemetry.update();

        //Use the Teleop initialization method
        astroGary.InitTele(hardwareMap);

        //Set toggle initial states
        boolean rtTogglePressed = false;
        boolean rtToggleReleased = true;
        boolean ltTogglePressed = false;
        boolean ltToggleReleased = true;

        //Tell drivers that initializing is now complete
        telemetry.setAutoClear(true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //indianaGary.myRelicArm.relicGrab.setPosition(indianaGary.myRelicArm.RELIC_GRAB_OPEN);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Latch Height: ", astroGary.myLanderLatcher.motorLanderLift.getCurrentPosition());
            telemetry.update();

            //Pass controls to the drive control method.
            astroGary.drive.DriveControl(
                    BotControls.DriveYStick(this),
                    BotControls.DriveXStick(this),
                    BotControls.TurnStick(this),
                    BotControls.DriveThrottle(this));

            //LanderLatcher manual controls
            astroGary.myLanderLatcher.LifterControl(
                    BotControls.LanderLatchRaiseButton(this),
                    BotControls.LanderLatchLowerButton(this));
            astroGary.myLanderLatcher.LatchControl(
                    BotControls.LanderLatchHookOnButton(this),
                    BotControls.LanderLatchHookOffButton(this));
            telemetry.addData("Lifter Height: ", astroGary.myLanderLatcher.motorLanderLift.getCurrentPosition());

            //Mineral Grabber Controls
            astroGary.myMineralGrabber.SpinnerControl(
                    BotControls.SpinnerForwardButton(this),
                    BotControls.SpinnerStopButton(this),
                    BotControls.SpinnerBackwardButton(this));

            if (BotControls.MineralGrabberCollectButton(this)){
                astroGary.myMineralGrabber.CollectMode();
            }
            if (BotControls.MineralGrabberFoldButton(this)){
                astroGary.myMineralGrabber.DriveMode();
            }


            //Glyph Grabber Control
            //if (!astroGary.myGlyphLifter.GRAB_LOCKED) { //only allow control of grabber if not locked.
//                indianaGary.myGlyphLifter.GrabberControl(BotControls.GlyphGrabTrigger(this));
            //}

//            if (rtTogglePressed) {
//                rtToggleReleased = false;
//            } else {
                //Track when toggle button is released
//                rtToggleReleased = true;
//            }
//            rtTogglePressed = BotControls.GlyphGrabLockButton(this);
//            if (rtToggleReleased) { //only do something if toggle is pressed after being released
//                if (BotControls.GlyphGrabLockButton(this) && !indianaGary.myGlyphLifter.GRAB_LOCKED){
//                    indianaGary.myGlyphLifter.Grab();
//                } else {
//                    if (BotControls.GlyphGrabLockButton(this) && indianaGary.myGlyphLifter.GRAB_LOCKED) {
//                        indianaGary.myGlyphLifter.Release();
//                    }
//                }
//          }


        }
    }
}