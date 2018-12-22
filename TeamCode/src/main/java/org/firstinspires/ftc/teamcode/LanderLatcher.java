package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LanderLatcher {
    HardwareMap myHWMap;
    public Servo servoHookLander = null;
    public DcMotor motorLanderLatch = null;
    private double LATCHED = 0.5;
    private double UNLATCHED = 0.0;

    public void LanderLatcher() { //constructor
    }

    public void init(HardwareMap myNewHWMap) {
        myHWMap = myNewHWMap;

        //Hook Initialization
        servoHookLander = myHWMap.servo.get("servoHookLander");
        servoHookLander.setDirection(Servo.Direction.REVERSE);
        servoHookLander.setPosition(LATCHED);

        //Lifter Initialization
        motorLanderLatch = myHWMap.dcMotor.get("motorLanderLatch");
        motorLanderLatch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLanderLatch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLanderLatch.setDirection(DcMotor.Direction.FORWARD);
        //POS_START = motorLift.getCurrentPosition();
    }

    public void LowerToGround() {

    }

}

