package org.baconeers.configurations;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.FgCommon.RobotConfiguration;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * It is assumed that there is a configuration that is currently activated on the robot controller
 * (run menu / Configure Robot ) with the same name as this class.
 * It is also assumed that the device names in the 'init()' method below are the same as the devices
 * named on the activated configuration on the robot.
 */
public class SmokeyBase extends RobotConfiguration {
    // Left and right is viewed in the frame of the robot with the front being the harvester

    // Drive motors
    public DcMotor driveLeft;
    public DcMotor driveRight;

    // Harvester motors
    public DcMotor harvesterPrimary;
    public DcMotor harvesterSecondary;

    // Other motors
    public DcMotor lift;
    public DcMotor launcher;

    // Color Sensor
    //public ColorSensor sorterColorSensor;


    /**
     * Assign your class instance variables to the saved device names in the hardware map
     *
     * @param hardwareMap
     * @param telemetry
     */
    @Override
    protected void init(HardwareMap hardwareMap, Telemetry telemetry) {

        setTelemetry(telemetry);

        driveLeft = (DcMotor) getHardwareOn("left1_drive", hardwareMap.dcMotor);
        //driveLeftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRight = (DcMotor) getHardwareOn("left2_drive", hardwareMap.dcMotor);

        harvesterPrimary = hardwareMap.dcMotor.get("motor_harvester");
        harvesterSecondary = hardwareMap.dcMotor.get("motor_particlelift");
        //harvesterSecondary.setDirection(DcMotorSimple.Direction.REVERSE);

        lift = hardwareMap.dcMotor.get("motor_lift");
        launcher = hardwareMap.dcMotor.get("motor_launcher");

        /**
         * Examples
         *
         * motor.setDirection(DcMotorSimple.Direction.REVERSE);
         * colorSensor = hardwareMap.colorSensor.get("color");
         * servo = hardwareMap.servo.get("servo");
         * crServo = hardwareMap.crservo.get("BackServo");
         * crServo.setDirection(CRServo.Direction.REVERSE);
         */
    }


    /**
     * Factory method for this class
     *
     * @param hardwareMap
     * @param telemetry
     * @return
     */
    public static SmokeyBase newConfig(HardwareMap hardwareMap, Telemetry telemetry) {

        SmokeyBase config = new SmokeyBase();
        config.init(hardwareMap, telemetry);
        return config;
    }


}
