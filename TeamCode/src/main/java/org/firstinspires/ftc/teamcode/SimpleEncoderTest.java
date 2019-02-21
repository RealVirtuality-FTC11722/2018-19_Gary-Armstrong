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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@TeleOp(name="Simple Encoder Test", group="Training")
//@Disabled
public class SimpleEncoderTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;
//    private DcMotor motorArmSwivel = null;
    private DcMotor motorArmLift = null;
    private DcMotor motorArmElbow = null;
    private DcMotor motorLanderLatch = null;

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
//        motorArmSwivel = hardwareMap.get(DcMotor.class, "motorArmSwivel");
        motorArmLift = hardwareMap.get(DcMotor.class, "motorArmLift");
        motorArmElbow = hardwareMap.get(DcMotor.class, "motorArmElbow");
        motorLanderLatch = hardwareMap.get (DcMotor.class, "motorLanderLatch");

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
//        motorArmSwivel.setDirection(DcMotor.Direction.FORWARD);
        motorArmLift.setDirection(DcMotor.Direction.FORWARD);
        motorArmElbow.setDirection(DcMotor.Direction.FORWARD);
        motorLanderLatch.setDirection(DcMotor.Direction.FORWARD);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorArmSwivel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLanderLatch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorArmSwivel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArmLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArmElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLanderLatch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
//        motorArmSwivel.setPower(0);
        motorArmLift.setPower(0);
        motorArmElbow.setPower(0);
        motorLanderLatch.setPower(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Test controls
            motorFL.setPower(gamepad1.left_stick_y);
            motorFR.setPower(gamepad1.left_stick_y);
            motorBL.setPower(gamepad1.left_stick_y);
            motorBR.setPower(gamepad1.left_stick_y);
            if (gamepad1.dpad_up) {
                motorLanderLatch.setPower(1);
            } else if (gamepad1.dpad_down) {
                motorLanderLatch.setPower(-1);
            } else {
                motorLanderLatch.setPower(0);
            }
 //           motorArmSwivel.setPower(gamepad1.right_stick_x);
            motorArmLift.setPower(gamepad1.right_stick_y);
            motorArmElbow.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

            //Output values
            telemetry.addData("motorFL Pos: ", motorFL.getCurrentPosition());
            telemetry.addData("motorFR Pos: ", motorFR.getCurrentPosition());
            telemetry.addData("motorBL Pos: ", motorBL.getCurrentPosition());
            telemetry.addData("motorBR Pos: ", motorBR.getCurrentPosition());
//            telemetry.addData("motorArmSwvl Pos: ", motorArmSwivel.getCurrentPosition());
            telemetry.addData("motorArmElbw Pos: ", motorArmElbow.getCurrentPosition());
            telemetry.addData("motorArmLift Pos: ", motorArmLift.getCurrentPosition());
            telemetry.addData("motorLndLtch Pos: ", motorLanderLatch.getCurrentPosition());
            telemetry.update();
        }
    }
}
