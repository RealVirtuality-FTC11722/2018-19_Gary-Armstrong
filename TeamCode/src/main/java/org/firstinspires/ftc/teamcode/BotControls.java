package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Lyesome on 2018-01-13.
 * This class is used to pass the gamepad controls to generalized methods that the other classes can use.
 * This allows all the controls to be defined in one location and makes it easier to change the controls if need.
 */

public class BotControls {

    public BotControls()  { // constructor
    }

    //GAMEPAD 1
    //Drive controls
    public static double  DriveYStick(LinearOpMode op)              {return op.gamepad1.left_stick_y; }
    public static double  DriveXStick(LinearOpMode op)              {return op.gamepad1.left_stick_x; }
    public static double  TurnStick(LinearOpMode op)                {return op.gamepad1.right_stick_x;}
    public static double  DriveThrottle(LinearOpMode op)            {return op.gamepad1.right_trigger;}

    //Lander Latch Controls
    public static boolean LanderLatchRaiseButton(LinearOpMode op)   {return op.gamepad1.dpad_up; }
    public static boolean LanderLatchLowerButton(LinearOpMode op)   {return op.gamepad1.dpad_down; }
    public static boolean LanderLatchHookOnButton(LinearOpMode op)  {return op.gamepad1.x; }
    public static boolean LanderLatchHookOffButton(LinearOpMode op) {return op.gamepad1.b; }

    //GAMEPAD 2
    //Mineral Grabber control
    //public static boolean SpinnerStopButton(LinearOpMode op)        {return  op.gamepad2.b;}
    public static boolean SpinnerForwardButton(LinearOpMode op)     {return  op.gamepad2.right_bumper;}
    public static boolean SpinnerBackwardButton(LinearOpMode op)    {return  op.gamepad2.left_bumper;}
    public static boolean MineralArmTurnRightButton(LinearOpMode op){return op.gamepad2.dpad_right;}
    public static boolean MineralArmTurnLeftButton(LinearOpMode op) {return op.gamepad2.dpad_left;}
    public static double  MineralArmForwardStick(LinearOpMode op)   {return  op.gamepad2.right_stick_y;}
    public static boolean MineralGrabberCollectButton(LinearOpMode op) {return op.gamepad2.y;}
    public static boolean MineralGrabberFoldButton(LinearOpMode op) {return  op.gamepad2.x;}
    public static boolean MineralGrabberScoreButton(LinearOpMode op) {return  op.gamepad2.a;}
    public static double  MineralLiftStick(LinearOpMode op) {return op.gamepad2.left_stick_y;}
    public static double MineralElbowStick(LinearOpMode op) {return op.gamepad2.right_stick_y;}
    public  static double MineralWristDown(LinearOpMode op) {return op.gamepad2.right_trigger;}
    public  static double MineralWristUp(LinearOpMode op) {return op.gamepad2.left_trigger;}

}
