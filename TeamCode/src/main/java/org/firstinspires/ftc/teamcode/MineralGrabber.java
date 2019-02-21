package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MineralGrabber {
//    public DcMotor motorArmSwivel = null;
    public enum Mode {COLLECT_MODE, DRIVE_MODE, SCORE_MODE}
    public Mode armMode = Mode.DRIVE_MODE;
    public DcMotor motorArmLift = null;
    public DcMotor motorArmLift2 = null;
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
    public double WRIST_FOLD_POS = 0.37;
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

    private int liftPos = 0;
    private int liftPos2 = 0;
    private int elbowPos = 0;

    public void MineralGrabber() { //constructor
    }

    public void initMotors(HardwareMap myNewHWMap) {
  //      motorArmSwivel = myNewHWMap.get(DcMotor.class, "motorArmSwivel");
        motorArmLift = myNewHWMap.get(DcMotor.class, "motorArmLift");
        motorArmLift2 = myNewHWMap.get(DcMotor.class, "motorArmLift2");
        motorArmElbow = myNewHWMap.get(DcMotor.class, "motorArmElbow");

    //    motorArmSwivel.setDirection(DcMotor.Direction.FORWARD);
        motorArmLift.setDirection(DcMotor.Direction.FORWARD);
        motorArmLift2.setDirection(DcMotor.Direction.FORWARD);
        motorArmElbow.setDirection(DcMotor.Direction.FORWARD);


        motorArmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArmLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArmElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      //  motorArmSwivel.setPower(0);
        motorArmLift.setPower(0);
        motorArmLift2.setPower(0);
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
        double LIFT_POWER = 1;
        double ELBOW_POWER = 0.5;
        double SWIVEL_POWER = 0.20;

        if (swivelStick > 0.2 || swivelStick < 0.2) {
        //    motorArmSwivel.setPower(swivelStick * SWIVEL_POWER);
        }

//        if (liftStick < 0){
//            motorArmLift.setTargetPosition(liftPos + 1);
//            motorArmLift2.setTargetPosition(liftPos2 + 1);
//        }
//        else if (liftStick > 0){
//            motorArmLift.setTargetPosition(liftPos - 1);
//            motorArmLift2.setTargetPosition(liftPos2 - 1);
//        } else {
//            motorArmLift.setTargetPosition(liftPos);
//            motorArmLift2.setTargetPosition(liftPos2);
//        }
//        motorArmLift.setPower(1);
//        motorArmLift2.setPower(1);
        if (liftStick != 0) {
            motorArmLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorArmLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorArmLift.setPower(-liftStick * 0.1);
            motorArmLift2.setPower(-liftStick * 0.1);
            liftPos = motorArmLift.getCurrentPosition();
            liftPos2 = motorArmLift2.getCurrentPosition();
        } else {
            motorArmLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorArmLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorArmLift.setTargetPosition(liftPos);
            motorArmLift2.setTargetPosition(liftPos2);
            motorArmLift.setPower(0.6);
            motorArmLift2.setPower(0.6);
        }

        if (elbowStick != 0) {
            motorArmElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorArmElbow.setPower(elbowStick * 0.1);
            elbowPos = motorArmElbow.getCurrentPosition();
        } else {
            motorArmElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorArmElbow.setTargetPosition(elbowPos);
            motorArmElbow.setPower(0.6);
        }
        if (armMode == Mode.COLLECT_MODE) {
            servoArmWrist.setPosition(WRIST_COLLECT_POS + wristTrigger * 0.01);
        } else {
            servoArmWrist.setPosition(WRIST_FOLD_POS + wristTrigger * 0.1);
        }
    }

    //Raise Mineral Grabber to scoring height and stop Spinner
    public void ScoreMode() {
        armMode = Mode.SCORE_MODE;
        motorArmLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmLift.setTargetPosition(ARM_SCORE_POS);
        motorArmLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmLift2.setTargetPosition(ARM_SCORE_POS);
        motorArmElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmElbow.setTargetPosition(FOREARM_SCORE_POS);
        motorArmLift.setPower(0.02);
        motorArmElbow.setPower(0.02);
        servoArmWrist.setPosition(WRIST_SCORE_POS);
    }

    //Lower Mineral Grabber to ground and start Spinner
    public void CollectMode(LinearOpMode op) {
        armMode = Mode.COLLECT_MODE;
        motorArmLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Step 1
        motorArmLift.setTargetPosition(280);
        motorArmLift2.setTargetPosition(280);
        motorArmLift.setPower(0.1);
        motorArmLift2.setPower(0.1);
        op.sleep(2000);
        motorArmElbow.setTargetPosition(-59);
        motorArmElbow.setPower(0.1);
        op.sleep(2000);

        //Step 2
        motorArmLift.setTargetPosition(539);
        motorArmLift2.setTargetPosition(539);
        motorArmElbow.setTargetPosition(-147);
        motorArmLift.setPower(0.1);
        motorArmLift2.setPower(0.1);
        motorArmElbow.setPower(0.1);
        op.sleep(4000);


        //Step 3
        motorArmLift.setTargetPosition(629);
        motorArmLift2.setTargetPosition(629);
        motorArmElbow.setTargetPosition(-149);
        motorArmLift.setPower(0.1);
        motorArmLift2.setPower(0.1);
        motorArmElbow.setPower(0.1);

        servoArmWrist.setPosition(WRIST_COLLECT_POS);
        elbowPos = -149;
        liftPos = 629;
        liftPos2 = 629;

    }

    //Partially fold up mineral grabber for driving
    public void DriveMode(LinearOpMode op ) {
        armMode = Mode.DRIVE_MODE;
        motorArmLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Step 1
        servoArmWrist.setPosition(WRIST_FOLD_POS);
        motorArmLift.setTargetPosition(539);
        motorArmLift2.setTargetPosition(539);
        motorArmElbow.setTargetPosition(-147);
        motorArmLift.setPower(0.1);
        motorArmLift2.setPower(0.1);
        motorArmElbow.setPower(0.1);
        op.sleep(4000);

        //Step 2
        motorArmLift.setTargetPosition(273);
        motorArmLift2.setTargetPosition(273);
        motorArmElbow.setTargetPosition(-59);
        motorArmLift.setPower(0.1);
        motorArmLift2.setPower(0.1);
        motorArmElbow.setPower(0.1);
        op.sleep(3000);


        //Step 3
        motorArmLift.setTargetPosition(0);
        motorArmLift2.setTargetPosition(0);
        motorArmElbow.setTargetPosition(0);
        motorArmLift.setPower(0.1);
        motorArmLift2.setPower(0.1);
        motorArmElbow.setPower(0.1);

        elbowPos = 0;
        liftPos = 0;
        liftPos2 = 0;


    }

    public void Extend(boolean outBtn, boolean inBtn) {
        motorArmLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArmLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArmElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (outBtn) {
            motorArmLift.setTargetPosition(motorArmLift.getCurrentPosition()+1);
            motorArmLift2.setTargetPosition(motorArmLift.getCurrentPosition()+1);
            motorArmElbow.setTargetPosition(motorArmElbow.getCurrentPosition()+1);
        }
        else if (inBtn) {
            motorArmLift.setTargetPosition(motorArmLift.getCurrentPosition()-1);
            motorArmLift2.setTargetPosition(motorArmLift.getCurrentPosition()-1);
            motorArmElbow.setTargetPosition(motorArmElbow.getCurrentPosition()-1);
        }
        else {

        }
        motorArmLift.setPower(0.5);
        motorArmLift2.setPower(0.5);
        motorArmElbow.setPower(0.5);
    }

    public void Raise(boolean raiseBtn, boolean lowerBtn) {

    }

    //Unfold Mineral Grabber, reverse Spinner to drop off Team Marker
    public void DropTeamMarker(LinearOpMode op) {
//        CollectMode();
        servoArmSpinner1.setPower(SPIN_BACKWARD);
        servoArmSpinner2.setPower(SPIN_BACKWARD);
        op.sleep(2000);
        servoArmSpinner1.setPower(CRSERVO_STOP);
        servoArmSpinner2.setPower(CRSERVO_STOP);
//        DriveMode();
    }

}
