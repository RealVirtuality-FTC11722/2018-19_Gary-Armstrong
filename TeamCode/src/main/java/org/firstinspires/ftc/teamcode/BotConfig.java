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
    MineralGrabber myMineralGrabber = new MineralGrabber();
    LanderLatcher myLanderLatcher = new LanderLatcher();
    Sensors mySensors = new Sensors();

    public BotConfig() { // constructor
    }

    //Method to initialize all the Hardware
    public void InitAuto(HardwareMap myNewHWMap){
        //Initialize Servos first to minimize movement
            myMineralGrabber.initServos(myNewHWMap);
            myLanderLatcher.initServos(myNewHWMap);
        //Then initialize sensors
            //mySensors.initSensors(myNewHWMap);
           //myVuMark.init(myNewHWMap);
        //Finally initialize motors
            myMineralGrabber.initMotors(myNewHWMap);
            myLanderLatcher.initMotors(myNewHWMap);
            drive.initAuto(myNewHWMap);
    }
    public void InitTele(HardwareMap myNewHWMap){
        //Initialize Servos first to minimize movement
            myMineralGrabber.initServos(myNewHWMap);
            myLanderLatcher.initServos(myNewHWMap);
        //initialize sensors
            //mySensors.initSensors(myNewHWMap);
        //Then initialize motors
            myMineralGrabber.initMotors(myNewHWMap);
            myLanderLatcher.initMotors(myNewHWMap);
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