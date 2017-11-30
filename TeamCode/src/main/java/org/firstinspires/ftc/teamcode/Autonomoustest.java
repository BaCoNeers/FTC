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

@Autonomous(name = "Autonomoustest")
public class Autonomoustest extends LinearOpMode {
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


        telemetry.log().add("current position 1 %d", motor1.getCurrentPosition());
        telemetry.log().add("current position 2 %d", motor2.getCurrentPosition());

        telemetry.log().add("before wait for start");
        waitForStart();
        //DriveForward(0.5,5000);
        turn(90,Turn.LEFT);
        turn(90,Turn.LEFT);
        //DriveForward(0.5,5000);
        turn(90,Turn.RIGHT);
        turn(90,Turn.RIGHT);

        Thread.sleep(3000);
    }

    public float startAngle;

    public void turn(float degrestoturn, Turn direction) {
        turn(degrestoturn, direction, 5);
    }

    public void turn(float degrestoturn, Turn direction, float tolerance) {
        telemetry.setAutoClear(false);
        Telemetry.Item angItem = telemetry.addData("angle", "%12.3f", 0.0);
        Telemetry.Item teltargetangle = telemetry.addData("target angle", "%12.3f", 0.0);
        telemetry.log().add("in turn");
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;
        float offset = 0;
        float targetangle;
        telemetry.log().add("result test: %f", heading - degrestoturn);
        if (direction.equals(Turn.LEFT)) {
            targetangle = heading - degrestoturn;
            telemetry.log().add("target angle: %f", targetangle);
            telemetry.log().add("degrestoturn: %f", degrestoturn);
            telemetry.log().add("heading: %f", heading);
            telemetry.log().add("result test: %f", heading - degrestoturn);
        } else {
            targetangle = heading + degrestoturn;
            telemetry.log().add("target angle: %f", targetangle);
            telemetry.log().add("degrestoturn: %f", degrestoturn);
            telemetry.log().add("heading: %f", heading);
            telemetry.log().add("result test: %f", heading - degrestoturn);
        }
        telemetry.log().add("heading: %f", heading);
        telemetry.log().add("target3: %f", targetangle);
        if (targetangle > 160) {
            offset = -160;
            targetangle += offset;
        } else if (targetangle < -160) {
            offset = 160;
            targetangle += offset;
        }
        heading += offset;
        int count = 0;

        float diff = heading - targetangle;
        while (Math.abs(diff) > tolerance && count < 1000000000) {
            telemetry.log().add("count: %d", count);
            telemetry.log().add("offset: %f", offset);
            telemetry.log().add("diff: %f", diff);
            telemetry.log().add("heading: %f", heading);
            telemetry.log().add("target: %f", targetangle);

            count++;
            if (diff < 0) {
                motor1.setPower(0.21);
                motor2.setPower(-0.21);
            } else {
                motor1.setPower(-0.21);
                motor2.setPower(0.21);
            }

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle + offset;
            diff = heading - targetangle;
            angItem.setValue("%12.3f", heading);

        }

            motor1.setPower(0);
            motor2.setPower(0);
        }

        public void DriveForward(int distance)
        {
            DriveForward(1, distance);
        }

        public void DriveForward(double power, int distance)
        {

            boolean reachedmotor1 = false;
            boolean reachedmotor2= false;

            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motor1.setPower(power);
            motor2.setPower(power);


            int loopcount = 0;

            while (!reachedmotor1 && !reachedmotor2)
                {
                loopcount++;

                telemetry.log().add("in loop %d",loopcount);
                telemetry.log().add("current position 1 %d", motor1.getCurrentPosition());
                telemetry.log().add("current position 2 %d", motor2.getCurrentPosition());




                if (motor1.getCurrentPosition() > distance)
                {
                    motor1.setPower(0);
                    reachedmotor1 = true;
                }
                if (motor2.getCurrentPosition() > distance)
                {
                    motor2.setPower(0);
                    reachedmotor2 = true;
                }

            }
            motor1.setPower(0);
            motor2.setPower(0);

            motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }



