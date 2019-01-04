package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LanderLatcher {
    HardwareMap myHWMap;
    public Servo servoLanderHook = null;
    public DcMotor motorLanderLift = null;
    public int LIFT_MAX_POS = 10;
    public int LIFT_MIN_POS = 0;
    private double LATCHED = 0.5;
    private double UNLATCHED = 0.0;

    public void LanderLatcher() { //constructor
    }

    public void init(HardwareMap myNewHWMap) {
        myHWMap = myNewHWMap;

        //Hook Initialization
        servoLanderHook = myHWMap.servo.get("servoHookLander");
        servoLanderHook.setDirection(Servo.Direction.REVERSE);
        servoLanderHook.setPosition(LATCHED);

        //Lifter Initialization
        motorLanderLift = myHWMap.dcMotor.get("motorLanderLatch");
        motorLanderLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLanderLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLanderLift.setDirection(DcMotor.Direction.FORWARD);
        //POS_START = motorLift.getCurrentPosition();
    }

    public void LowerToGround() {
        motorLanderLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLanderLift.setTargetPosition(LIFT_MAX_POS);
        //Don't Unlatch until target position is reached
        if (motorLanderLift.getCurrentPosition() >= LIFT_MAX_POS) {
            servoLanderHook.setPosition(UNLATCHED);
        }
    }

    public void LifterControl(boolean raiseBtn, boolean lowerBtn) {
        //Make sure Lifter motor is using encoder when going to preset positions
        //Do NOT reset the encoder or min and max positions will change
        motorLanderLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (raiseBtn && (motorLanderLift.getCurrentPosition() <= LIFT_MAX_POS)) {
            motorLanderLift.setPower(1.0);
        }
        else if (lowerBtn && (motorLanderLift.getCurrentPosition() >= LIFT_MIN_POS)) {
            motorLanderLift.setPower(-1.0);
        }
        else {
            motorLanderLift.setPower(0);
        }

    }

}

