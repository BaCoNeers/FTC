package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.R;

/**
 * Autonomous program being used for test
 */

@Autonomous(name = "Autonomoustest")
public class Autonomoustest extends LinearOpMode {
    // Declare drive motors
    Configuration robot = new Configuration();
    Orientation angles;
    public float heading;




    // Constants for moving arm

    @Override
    public void runOpMode() throws InterruptedException {

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        params.vuforiaLicenseKey = "AQQN8vT/////AAAAGR2kWcTZSEaGsNmfOFgjVCpfRMD0rrC8iVwL5YiD9FQny/LDfDTPHuZMkS31CZvPgOpu9GPC10zAHbs2om9lY3IZmlQ944EDdEeCFkzTFlN5Fk1/gzwUbMgR1+8qwBy/7FsoQOgXFApTWMRfogt6FqXahm7g0gpfzDiOhAPHgHmMDYL5wqHdBgRdt12rT6FnwePm7H3Z7hcEPh7BwLoD8wFa9mqhDnNkm2czsZLiGgQQGy3bdWY3kq3Hzn6XNDREjq4xk2RmTMWZi6BFDZgFAMaaTT2PdLoF6waMR+o21FW/EHCRd1fJu1fNPSvtyLdwxkUG+JrjVtTBBQGrQ5mHRuZ/Bp0XlHijhW0KEh6/G7lb";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables cyper = vuforia.loadTrackablesFromAsset("RelicVuMark");
        cyper.get(0).setName("0");
        cyper.get(1).setName("1");
        cyper.get(2).setName("3");
        cyper.activate();

        robot.leftDrive.resetDeviceConfigurationForOpMode();
        robot.rightDrive.resetDeviceConfigurationForOpMode();
        robot.leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 10);


        telemetry.log().add("current position 1 %d", robot.leftDrive.getCurrentPosition());
        telemetry.log().add("current position 2 %d", robot.rightDrive.getCurrentPosition());



        telemetry.log().add("before wait for start");
        waitForStart();
        Jewel(true);
//        DriveForward(0.5,10000);
//        turn(90,Turn.LEFT);
//        DriveForward(0.5,10000);

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
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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
            telemetry.log().add("distance: %f" , degrestoturn);
            if (direction == Turn.RIGHT)  telemetry.log().add("turn: right");
            if (direction == Turn.LEFT) telemetry.log().add("turn: Left");

            count++;
            if (diff < 0) {
                robot.leftDrive.setPower(0.21);
                robot.rightDrive.setPower(-0.21);
            } else {
                robot.leftDrive.setPower(-0.21);
                robot.rightDrive.setPower(0.21);
            }

            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle + offset;
            diff = heading - targetangle;
            angItem.setValue("%12.3f", heading);

        }

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        }

        public void DriveForward(int distance)
        {
            DriveForward(1, distance);
        }

        public void DriveForward(double power, int distance)
        {

            boolean reachedmotor1 = false;
            boolean reachedmotor2 = false;

            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




            robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.leftDrive.setPower(-power);
            robot.rightDrive.setPower(-power);


            int loopcount = 0;

            while (reachedmotor1 == false || reachedmotor2 == false)
                {
                loopcount++;



                telemetry.log().add("in loop %d",loopcount);
                telemetry.log().add("current position 1 %d", robot.leftDrive.getCurrentPosition());
                telemetry.log().add("current position 2 %d", robot.rightDrive.getCurrentPosition());
                telemetry.log().add("reached motor 1 : %b", reachedmotor1);
                telemetry.log().add("reached motor 2 : %b", reachedmotor2);
                    telemetry.log().add("distance %d", distance);

                if (robot.leftDrive.getCurrentPosition() > distance)
                {
                    telemetry.log().add("motor2 > distance");
                    robot.leftDrive.setPower(0);
                    reachedmotor1 = true;
                }
                if (robot.rightDrive.getCurrentPosition() > distance)
                {
                    telemetry.log().add("motor2 > distance");
                    robot.rightDrive.setPower(0);
                    reachedmotor2 = true;
                }


            }
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        }


        public void Jewel(boolean toggle){
            robot.drop.setPosition(1);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if (robot.drop.getPosition() == 1 && (robot.Colour.blue() > 0 || robot.Colour.red() > 0)) {
                if (toggle) {
                    if (robot.Colour.blue() > robot.Colour.red()) {
                        robot.push.setPosition(1);
                    } else {
                        robot.push.setPosition(0);
                    }
                } else {
                    if (robot.Colour.red() > robot.Colour.blue()) {
                        robot.push.setPosition(1);
                    } else {
                        robot.push.setPosition(0);
                    }
                }
            }
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.drop.setPosition(0.5);
            robot.push.setPosition(0.45);


    }


}



