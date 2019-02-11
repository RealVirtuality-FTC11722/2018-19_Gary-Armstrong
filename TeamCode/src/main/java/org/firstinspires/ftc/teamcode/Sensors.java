package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class Sensors {
    HardwareMap myHWMap;
    BNO055IMU imu; // I AM YOU GYRO
    ModernRoboticsI2cRangeSensor rangeSensorF;
    ModernRoboticsI2cRangeSensor rangeSensorS;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public void Sensors() { // constructor
    }

    public void initSensors(HardwareMap MyHWMap) {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = myHWMap.get(BNO055IMU.class, "iamgyro");

        rangeSensorF = myHWMap.get(ModernRoboticsI2cRangeSensor.class, "iamsensor_rangeF");
        rangeSensorS = myHWMap.get(ModernRoboticsI2cRangeSensor.class, "iamsensor_rangeS");
        imu.initialize(parameters);

        double level = imu.getAngularOrientation().secondAngle + 100;

    }

}
