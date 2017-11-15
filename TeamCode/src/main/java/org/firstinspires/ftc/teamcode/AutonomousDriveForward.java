package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Autonomous program being used for test
 */

@Autonomous(name = "Autonomoustest Forward")
public class AutonomousDriveForward extends LinearOpMode {
    // Declare drive motors

    BNO055IMU imu;
    Orientation angles;
    public float heading;
    private DcMotor motor1;
    private DcMotor motor2;
    public float difference;
    public boolean anglereached;


    // Constants for moving arm

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drive motors


        // If drive motors are given full power, robot would spin because of the motors being in
        // opposite directions. So reverse one


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        // parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        motor1 = hardwareMap.dcMotor.get("driveRight");
        motor2 = hardwareMap.dcMotor.get("driveLeft");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setPower(0);
        motor2.setPower(0);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);

        telemetry.log().add("before wait for start");
        waitForStart();
        forward(400);

        Thread.sleep(2000);

    }

    public float startAngle;

    public void forward(int distancetostop) {
        forward(distancetostop);
    }

    public void forawrd(int distancetostop) {

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(1);
        motor2.setPower(1);

        motor1.setTargetPosition(distancetostop);
        motor2.setTargetPosition(distancetostop);


        motor1.setPower(0);
       motor2.setPower(0);
//        if (direction == Turn.LEFT) {
//
//        }
//
//        if (degrestoturn < 0) {
//            targetangle = heading + degrestoturn;
//           if (targetangle < -180){
//                targetangle += 180;
//                targetangle = 180 + targetangle;
//            }
//            while (heading > targetangle + 10 && heading < targetangle - 10 ){
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                heading = angles.firstAngle;
//                angItem.setValue("%12.3f", heading);
//
//
//        }}
//            motor1.setPower(0);
//            motor2.setPower(0);
//
//        if (degrestoturn < 0) {
//            targetangle = heading + degrestoturn;
//            if (targetangle > 180){
//                targetangle -= 180;
//                targetangle = -180 + targetangle;
//            }
//            while (heading > targetangle + 10 && heading < targetangle - 10 ){
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                heading = angles.firstAngle;
//                angItem.setValue("%12.3f", heading);
//                motor1.setPower(-0.11);
//                motor2.setPower(0.11);
//            }}
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }



