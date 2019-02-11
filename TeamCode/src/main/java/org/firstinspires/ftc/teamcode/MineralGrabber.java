package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MineralGrabber {
    public DcMotor motorArmSwivel = null;
    public DcMotor motorArmLift = null;
    public DcMotor motorArmElbow = null;
    public Servo servoArmWrist = null;
    public CRServo servoArmSpinner1 = null;
    public CRServo servoArmSpinner2 = null;

    public double SWIVEL_POWER = 0.2;
    public double SWIVEL_LEFT_LIMIT = -1;
    public double SWIVEL_RIGHT_LIMIT = 1;
    public double MAX_ELBOW_POWER = 0.4;

    public double SPIN_FORWARD = 0.5;
    public double SPIN_BACKWARD = -0.5;
    public double CRSERVO_STOP = 0.0;
    public double WRIST_FOLD_POS = 0.38;
    public double WRIST_COLLECT_POS = 0.45;
    public double WRIST_SCORE_POS = 0.5;

    public double ARM_LENGTH = 1;
    public double ARM_MIN_ANGLE = 0;
    public double ARM_MAX_ANGLE = 90;
    public int ARM_FOLD_POS = 0;
    public int ARM_COLLECT_POS = 600;
    public int ARM_SCORE_POS = 0;

    public double FOREARM_LENGTH = 1;
    public double FOREARM_MIN_ANGLE = 0;
    public double FOREARM_MAX_ANGLE = 90;
    public int FOREARM_FOLD_POS = 0;
    public int FOREARM_COLLECT_POS = -400;
    public int FOREARM_SCORE_POS = 0;

    public void MineralGrabber() { //constructor
    }

    public void initMotors(HardwareMap myNewHWMap) {
        motorArmSwivel = myNewHWMap.get(DcMotor.class, "motorArmSwivel");
        motorArmLift = myNewHWMap.get(DcMotor.class, "motorArmLift");
        motorArmElbow = myNewHWMap.get(DcMotor.class, "motorArmElbow");

        motorArmSwivel.setDirection(DcMotor.Direction.FORWARD);
        motorArmLift.setDirection(DcMotor.Direction.REVERSE);
        motorArmElbow.setDirection(DcMotor.Direction.FORWARD);

        motorArmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArmElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorArmSwivel.setPower(0);
        motorArmLift.setPower(0);
        motorArmElbow.setPower(0);
    }

    public void initServos(HardwareMap myNewHWMap) {
        servoArmWrist = myNewHWMap.get(Servo.class, "servoArmWrist");
        servoArmSpinner1 = myNewHWMap.get (CRServo.class, "servoArmSpinner1");
        servoArmSpinner2 = myNewHWMap.get (CRServo.class, "servoArmSpinner2");

        servoArmSpinner1.setDirection(CRServo.Direction.FORWARD);
        servoArmSpinner2.setDirection(CRServo.Direction.REVERSE);
        servoArmWrist.setDirection(Servo.Direction.FORWARD);

        servoArmWrist.setPosition(WRIST_FOLD_POS);
        servoArmSpinner1.setPower(CRSERVO_STOP);
        servoArmSpinner2.setPower(CRSERVO_STOP);
    }

    public void SpinnerControl(boolean fowardBtn, boolean backwardBtn) {
        if (fowardBtn) {
            servoArmSpinner1.setPower(SPIN_FORWARD);
            servoArmSpinner2.setPower(SPIN_FORWARD);
        }
        else if (backwardBtn) {
            servoArmSpinner1.setPower(SPIN_BACKWARD);
            servoArmSpinner2.setPower(SPIN_BACKWARD);
        }
        else {
            servoArmSpinner1.setPower(CRSERVO_STOP);
            servoArmSpinner2.setPower(CRSERVO_STOP);
        }


    }

    public void ManualArmControl(double swivelStick, double liftStick, double elbowStick, double wristTrigger) {
        motorArmLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double LIFT_POWER = 0.5;
        double ELBOW_POWER = 0.35;
        double SWIVEL_POWER = 0.20;
        int liftPos;
        int elbowPos;
        liftPos = motorArmLift.getCurrentPosition();
        if (liftStick < 0){
            motorArmLift.setTargetPosition(liftPos - 15);
        }
        else if (liftStick > 0){
            motorArmLift.setTargetPosition(liftPos + 15);
        } else {
            motorArmLift.setTargetPosition(liftPos);
        }
        motorArmLift.setPower(LIFT_POWER);
//        motorArmLift.setPower(liftStick*LIFT_POWER);
        motorArmSwivel.setPower(swivelStick*SWIVEL_POWER);
        elbowPos = motorArmElbow.getCurrentPosition();
        if (elbowStick < 0){
            motorArmElbow.setTargetPosition(elbowPos - 10);
        }
        else if (elbowStick > 0){
            motorArmElbow.setTargetPosition(elbowPos + 10);
        } else {
            motorArmElbow.setTargetPosition(elbowPos);
        }

//        if (elbowStick > 0) {
//            motorArmElbow.setPower(elbowStick * ELBOW_POWER*0.7);
//        } else {
//            motorArmElbow.setPower(elbowStick * ELBOW_POWER);
//        }
        servoArmWrist.setPosition(WRIST_FOLD_POS + wristTrigger*0.1);
    }

    //Raise Mineral Grabber to scoring height and stop Spinner
    public void ScoreMode() {
        motorArmLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmLift.setTargetPosition(ARM_SCORE_POS);
        motorArmElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmElbow.setTargetPosition(FOREARM_SCORE_POS);
        motorArmLift.setPower(0.02);
        motorArmElbow.setPower(0.02);
        servoArmWrist.setPosition(WRIST_SCORE_POS);
    }

    //Lower Mineral Grabber to ground and start Spinner
    public void CollectMode() {
        motorArmLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmLift.setTargetPosition(ARM_COLLECT_POS);
        motorArmElbow.setTargetPosition(FOREARM_COLLECT_POS);
        motorArmLift.setPower(0.02);
        motorArmElbow.setPower(0.02);
        servoArmWrist.setPosition(WRIST_COLLECT_POS);
    }

    //Partially fold up mineral grabber for driving
    public void DriveMode() {
        motorArmLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmLift.setTargetPosition(ARM_FOLD_POS);
        motorArmElbow.setTargetPosition(FOREARM_FOLD_POS);
        motorArmLift.setPower(0.03);
        motorArmElbow.setPower(0.03);
        servoArmWrist.setPosition(WRIST_FOLD_POS);
    }

    public void Extend(boolean outBtn, boolean inBtn) {
        motorArmLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArmElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public void DropTeamMarker(LinearOpMode op) {
        CollectMode();
        servoArmSpinner1.setPower(SPIN_BACKWARD);
        servoArmSpinner2.setPower(SPIN_BACKWARD);
        op.sleep(2000);
        servoArmSpinner1.setPower(CRSERVO_STOP);
        servoArmSpinner2.setPower(CRSERVO_STOP);
        DriveMode();
    }
}
