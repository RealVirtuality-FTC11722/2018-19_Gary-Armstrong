package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MineralGrabber {
    public DcMotor motorArmSwivel = null;
    public DcMotor motorArmLift = null;
    public DcMotor motorArmElbow = null;
    public Servo servoArmWrist = null;
    public CRServo servoArmSpinner = null;

    public double SWIVEL_POWER = 0.2;
    public double SWIVEL_LEFT_LIMIT = -1;
    public double SWIVEL_RIGHT_LIMIT = 1;
    public double MAX_ELBOW_POWER = 0.4;

    public double SPIN_FORWARD = 0.7;
    public double SPIN_BACKWARD = 0.3;
    public double CRSERVO_STOP = 0.5;
    public int WRIST_FOLD_POS = 0;
    public int WRIST_COLLECT_POS = 0;
    public int WRIST_SCORE_POS = 0;

    public double ARM_LENGTH = 1;
    public double ARM_MIN_ANGLE = 0;
    public double ARM_MAX_ANGLE = 90;
    public int ARM_FOLD_POS = 0;
    public int ARM_COLLECT_POS = 0;
    public int ARM_SCORE_POS = 0;

    public double FOREARM_LENGTH = 1;
    public double FOREARM_MIN_ANGLE = 0;
    public double FOREARM_MAX_ANGLE = 90;
    public int FOREARM_FOLD_POS = 0;
    public int FOREARM_COLLECT_POS = 0;
    public int FOREARM_SCORE_POS = 0;

    public void MineralGrabber() { //constructor
    }

    public void initMotors(HardwareMap myNewHWMap) {
        motorArmSwivel = myNewHWMap.get(DcMotor.class, "motorArmSwivel");
        motorArmLift = myNewHWMap.get(DcMotor.class, "motorArmLift");
        motorArmElbow = myNewHWMap.get(DcMotor.class, "motorArmElbow");

        motorArmSwivel.setDirection(DcMotor.Direction.FORWARD);
        motorArmLift.setDirection(DcMotor.Direction.FORWARD);
        motorArmElbow.setDirection(DcMotor.Direction.FORWARD);

        motorArmSwivel.setPower(0);
        motorArmLift.setPower(0);
        motorArmElbow.setPower(0);
    }

    public void initServos(HardwareMap myNewHWMap) {
        servoArmWrist = myNewHWMap.get(Servo.class, "servoArmWrist");
        servoArmSpinner = myNewHWMap.get (CRServo.class, "servoArmSpinner");

        servoArmSpinner.setDirection(CRServo.Direction.FORWARD);
        servoArmWrist.setDirection(Servo.Direction.FORWARD);

        servoArmWrist.setPosition(WRIST_FOLD_POS);
        servoArmSpinner.setPower(CRSERVO_STOP);
    }

    public void SpinnerControl(boolean fowardBtn, boolean stopBtn, boolean backwardBtn) {
        if (fowardBtn) {
            servoArmSpinner.setPower(SPIN_FORWARD);
        }

        if (stopBtn) {
            servoArmSpinner.setPower(CRSERVO_STOP);
        }

        if (backwardBtn) {
            servoArmSpinner.setPower(SPIN_BACKWARD);
        }

    }

    //Raise Mineral Grabber to scoring height and stop Spinner
    public void ScoreMode() {
        motorArmLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmLift.setTargetPosition(ARM_SCORE_POS);
        motorArmElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmElbow.setTargetPosition(FOREARM_SCORE_POS);
        motorArmLift.setPower(0.5);
        motorArmElbow.setPower(0.5);
        servoArmWrist.setPosition(WRIST_SCORE_POS);
    }

    //Lower Mineral Grabber to ground and start Spinner
    public void CollectMode() {
        motorArmLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmLift.setTargetPosition(ARM_COLLECT_POS);
        motorArmElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmElbow.setTargetPosition(FOREARM_COLLECT_POS);
        motorArmLift.setPower(0.5);
        motorArmElbow.setPower(0.5);
        servoArmWrist.setPosition(WRIST_COLLECT_POS);
    }

    //Partially fold up mineral grabber for driving
    public void DriveMode() {

    }

    public void Extend(boolean outBtn, boolean inBtn) {
        motorArmLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER;
        motorArmElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER;
        if (outBtn) {
            motorArmLift.setTargetPosition(motorArmLift.getCurrentPosition()+1);
            motorArmElbow.setTargetPosition(motorArmElbow.getCurrentPosition()+1);
        }
        else if (inBtn) {
            motorArmLift.setTargetPosition(motorArmLift.getCurrentPosition()-1);
            motorArmElbow.setTargetPosition(motorArmElbow.getCurrentPosition()-1);
        }
        else {

        }
        motorArmLift.setPower(0.5);
        motorArmElbow.setPower(0.5);
    }

    public void Raise(boolean raiseBtn, boolean lowerBtn) {

    }

    //Unfold Mineral Grabber, reverse Spinner to drop off Team Marker
    public void DropTeamMarker() {
        CollectMode();

    }
}
