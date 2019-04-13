package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * Created by Lyesome on 2018-01-13.
 * This file contains all the instructions for controlling the robot in Teleop mode.
 */

@TeleOp(name="Demo drive", group="Linear OpMode")  // @Autonomous(...) is the other common choice
//@Disabled



public class demodrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorB = null;
    private Servo Fsteer = null;

   // ModernRoboticsI2cRangeSensor rangeSensor;

    public void runOpMode() {

       // rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
       // telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
        //telemetry.addData("raw optical", rangeSensor.rawOptical());
        //telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
       // telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorB = hardwareMap.get(DcMotor.class, "motorB");
        Fsteer = hardwareMap.get(Servo.class, "Fsteer");

        motorB.setDirection(DcMotor.Direction.FORWARD);
        Fsteer.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            motorB.setPower(gamepad1.left_stick_y);

            Fsteer.setPosition(0.5+gamepad1.right_stick_x*0.5);
        }

    }
}


