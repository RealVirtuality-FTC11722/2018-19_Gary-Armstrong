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

            //Mineral Grabber Controls
            astroGary.myMineralGrabber.SpinnerControl(
                    BotControls.SpinnerForwardButton(this),
                    BotControls.SpinnerBackwardButton(this));
            astroGary.myMineralGrabber.ManualArmControl(
                    gamepad2.left_stick_x,
                    gamepad2.left_stick_y,
                    gamepad2.right_stick_y,
                    gamepad2.right_trigger);
            telemetry.addData("Wrist Position: ", astroGary.myMineralGrabber.servoArmWrist.getPosition());
            telemetry.addData("Elbow Pos: ", astroGary.myMineralGrabber.motorArmElbow.getCurrentPosition());
            telemetry.addData("Lift Arm Pos: ", astroGary.myMineralGrabber.motorArmLift.getCurrentPosition());
            telemetry.addData("Lift1 Arm Pow: ", astroGary.myMineralGrabber.motorArmLift.getPower());
            telemetry.addData("Lift2 Arm Pow: ", astroGary.myMineralGrabber.motorArmLift2.getPower());
            telemetry.addData("Spin1 Pwr: ", astroGary.myMineralGrabber.servoArmSpinner1.getPower());
            telemetry.addData("Spin2 Pwr: ", astroGary.myMineralGrabber.servoArmSpinner2.getPower());
            if (BotControls.MineralGrabberCollectButton(this)){
                astroGary.myMineralGrabber.CollectMode(this);
                telemetry.addData("Collect ", "Mode");
            }
            if (BotControls.MineralGrabberFoldButton(this)){
                astroGary.myMineralGrabber.DriveMode(this);
                telemetry.addData("Drive ", "Mode");
            }
            if (BotControls.MineralGrabberScoreButton(this)){
                //astroGary.myMineralGrabber.ScoreMode();
                telemetry.addData("Score ", "Mode");
            }
//            if (astroGary.mySensors.imu.getAngularOrientation().secondAngle > 3) {
//                astroGary.myMineralGrabber.servoArmWrist.setPosition(astroGary.myMineralGrabber.servoArmWrist.getPosition() - 0.002);
//            } else if (astroGary.mySensors.imu.getAngularOrientation().secondAngle < -3) {
//                astroGary.myMineralGrabber.servoArmWrist.setPosition(astroGary.myMineralGrabber.servoArmWrist.getPosition() + 0.002);
//            }

            telemetry.update();

        }

    }
}