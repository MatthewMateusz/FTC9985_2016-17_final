package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created on 11/25/2016.
 */

public class UltimateSetupSensors {

    /* Public sensor members. */
    /* Touch Sensor */
    public ModernRoboticsTouchSensor touchSensorFront    = null;
    public ModernRoboticsTouchSensor touchSensorArmPush  = null;
    public ModernRoboticsTouchSensor touchSensorArmIn    = null;
    public ModernRoboticsTouchSensor touchSensorArmOut   = null;

    //ODS *Addon
    public OpticalDistanceSensor lightSensor = null;

    //MRColor Sensor *Addon
    public ColorSensor colorSensor = null;

    // MRGyro sensor, NOTE: will be set back to NULL if problems with initialization or calibration
    public ModernRoboticsI2cGyro gyroSensor = null;

    public ModernRoboticsI2cRangeSensor LeftDistanceSensor = null;
    public ModernRoboticsI2cRangeSensor RightDistanceSensor = null;

    /* local members. */
    HardwareMap hwMap =  null;


    /* Constructor */
    public UltimateSetupSensors()
    {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        lightSensor = hwMap.opticalDistanceSensor.get("lightSensor");
        lightSensor.enableLed(true);

        LeftDistanceSensor = (ModernRoboticsI2cRangeSensor) hwMap.ultrasonicSensor.get("LeftSonicSensor");
        RightDistanceSensor = (ModernRoboticsI2cRangeSensor) hwMap.ultrasonicSensor.get("RightSonicSensor");




        //Define touchSensorFront
        //touchSensorFront    = (ModernRoboticsDigitalTouchSensor) hwMap.touchSensor.get("touchFront");



        // Define ColorSensor & enbale led
        //colorSensor = hwMap.colorSensor.get("sensor_color");
        //colorSensor.enableLed(false); // detect the shining light, not reflected

        // get a reference to a Modern Robotics GyroSensor object if available
        try
        {
            gyroSensor = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");
            gyroSensor.resetZAxisIntegrator();
        }
        catch (Exception e)
        {
            gyroSensor = null;
        }
    }

}
