/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Simple Drive Test", group="Training")
//@Disabled
public class SimpleDriveTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;
    private DcMotor motorArmSwivel = null;
    private DcMotor motorArmLift = null;
    private DcMotor motorArmElbow = null;
    private Servo servoArmWrist = null;
    private CRServo servoArmSpinner = null;
    private Servo servoHookLander = null;
    private DcMotor motorLanderLatch = null;
    private double START_POSITION = 0.4;
    private double SWIVEL_POWER = 0.2;
    private double MAX_LIFT_POWER = 0.2;
    private double MAX_ELBOW_POWER = 0.4;
    private double MIN_THROTTLE = 0.3;
    private double SPIN_FORWARD = 0.7;
    private double SPIN_BACKWARD = 0.3;
    private double LATCHED = 0.4;
    private double UNLATCHED = 0.0;
    //private Servo grabby = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorFL  = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR  = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL  = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR  = hardwareMap.get(DcMotor.class, "motorBR");
        motorArmSwivel = hardwareMap.get(DcMotor.class, "motorArmSwivel");
        motorArmLift = hardwareMap.get(DcMotor.class, "motorArmLift");
        motorArmElbow = hardwareMap.get(DcMotor.class, "motorArmElbow");
        servoArmWrist = hardwareMap.get(Servo.class, "servoArmWrist");
        servoArmSpinner = hardwareMap.get (CRServo.class, "servoArmSpinner");
        servoHookLander = hardwareMap.get (Servo.class, "servoHookLander");
        motorLanderLatch = hardwareMap.get (DcMotor.class, "motorLanderLatch");
        //grabby  = hardwareMap.get(Servo.class, "grabby");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArmSwivel.setDirection(DcMotor.Direction.FORWARD);
        motorArmLift.setDirection(DcMotor.Direction.FORWARD);
        motorArmElbow.setDirection(DcMotor.Direction.FORWARD);
        servoArmSpinner.setDirection(CRServo.Direction.FORWARD);
        servoArmWrist.setDirection(Servo.Direction.FORWARD);
        servoHookLander.setDirection(Servo.Direction.REVERSE);
        motorLanderLatch.setDirection(DcMotor.Direction.FORWARD);
        //grabby.setDirection(Servo.Direction.FORWARD);

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorArmSwivel.setPower(0);
        motorArmLift.setPower(0);
        motorArmElbow.setPower(0);
        servoArmWrist.setPosition(START_POSITION);
        servoArmSpinner.setPower(0.5);
        servoHookLander.setPosition(0.5);
        motorLanderLatch.setPower(0);
        //grabby.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry


            // Send calculated power to wheels
            double grabPosition = 0;
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            //Set minimum throttle value so the trigger does not need to be pressed to drive
            double throttle = 1 - gamepad1.right_trigger * (1-MIN_THROTTLE);
            //double trottle = trigger * (1-DRIVE_POWER_MAX_LOW) + DRIVE_POWER_MAX_LOW;
            //Cube the value of turnstick so there's more control over low turn speeds
            double rightX = Math.pow(gamepad1.right_stick_x, 3);
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            motorFL.setPower(gamepad1.left_stick_y);
            motorFR.setPower(gamepad1.left_stick_y);
            motorBL.setPower(gamepad1.left_stick_y);
            motorBR.setPower(gamepad1.left_stick_y);

            if (gamepad1.dpad_up) {
                motorLanderLatch.setPower(1.0);
            }
            else if (gamepad1.dpad_down) {
                motorLanderLatch.setPower(-1.0);
            }
            else {
                motorLanderLatch.setPower(0);
            }
            if (gamepad1.x) {
                servoHookLander.setPosition(LATCHED);
            }
            if (gamepad1.b) {
                servoHookLander.setPosition(UNLATCHED);
            }

            if (gamepad2.dpad_left){
                motorArmSwivel.setPower(-SWIVEL_POWER);
            }
            else if (gamepad2.dpad_right) {
                 motorArmSwivel.setPower(SWIVEL_POWER);
            }
            else{
                 motorArmSwivel.setPower(0);
            }

            motorArmLift.setPower(gamepad2.right_stick_y * MAX_LIFT_POWER);

            motorArmElbow.setPower(gamepad2.right_trigger * MAX_ELBOW_POWER);
            motorArmElbow.setPower(- gamepad2.left_trigger * MAX_ELBOW_POWER);

            servoArmWrist.setPosition(0.3 + Range.clip(gamepad2.left_stick_y, -0.3, 0.7));
            telemetry.addData("ServoWrist: ", servoArmWrist.getPosition());

            if (gamepad2.a) {
                servoArmSpinner.setPower(SPIN_FORWARD);
            }

            if (gamepad2.b) {
                servoArmSpinner.setPower(0.5);
            }

            if (gamepad2.y) {
                servoArmSpinner.setPower(SPIN_BACKWARD);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Grabber: ", grabPosition);
            telemetry.update();
        }
    }
}
