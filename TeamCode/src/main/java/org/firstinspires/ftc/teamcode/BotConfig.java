package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Lyesome on 2018-01-13.
 * This class is used to initialize all of the components that make up the robot.
 */

public class BotConfig {
    //Add components to robot build
    MecanumDrive drive = new MecanumDrive();

    //VuMarkDecoder myVuMark = new VuMarkDecoder();

    //GlyphLifter myGlyphLifter = new GlyphLifter();
    MineralGrabber myMineralGrabber = new MineralGrabber();
    LanderLatcher myLanderLatcher = new LanderLatcher();

    public BotConfig() { // constructor
    }

    //Method to initialize all the Hardware
    public void InitAuto(HardwareMap myNewHWMap){
        //myGlyphLifter.init(myNewHWMap);
        //myMineralGrabber.init(myNewHWMap);
        //myLanderLatcher.init(myNewHWMap);
        //myVuMark.init(myNewHWMap);
        drive.initAuto(myNewHWMap);
    }
    public void InitTele(HardwareMap myNewHWMap){
        //myGlyphLifter.init(myNewHWMap);
        //myMineralGrabber.init(myNewHWMap);
        //myLanderLatcher.init(myNewHWMap);
        drive.initTele(myNewHWMap);
    }

    public void InitServos(HardwareMap myNewHWMap){
        //myGlyphLifter.initServos(myNewHWMap);
        myMineralGrabber.initServos(myNewHWMap);
        myLanderLatcher.initServos(myNewHWMap);
    }
    public void InitMotors(HardwareMap myNewHWMap){
        //myGlyphLifter.initMotors(myNewHWMap);
        myMineralGrabber.initMotors(myNewHWMap);
        myLanderLatcher.initMotors(myNewHWMap);
        drive.initMotors(myNewHWMap);
    }
    public void InitSensors(HardwareMap myNewHWMap){
        drive.initGyro(myNewHWMap);
        //myVuMark.init(myNewHWMap);
    }

}