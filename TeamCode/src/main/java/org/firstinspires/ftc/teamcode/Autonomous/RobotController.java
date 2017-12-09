package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Configuration;

/**
 * Created by me on 6/12/2017.
 */

public class RobotController {
    private static AutonomousOpMode autonomous;
    private static Telemetry telemetry;
    private static Configuration robot;
    private static float startAngle;

    //gets and sets the robots configuration and telemetry to be used later
    public static void RegisterAutonomousController(AutonomousOpMode _autonomous) {

        autonomous = _autonomous;
        telemetry = _autonomous.telemetry;
        robot = _autonomous.robot;

    }


    //set a defult value of tolerance if one is not passed though for the turning function
    public static void turn(float degrestoturn, Turn direction) {
        turn(degrestoturn, direction, 5);
    }

    //turning function
    public static void turn(float degrestoturn, Turn direction, float tolerance) {

        Orientation angles;
        float heading;

        telemetry.setAutoClear(false);
        Telemetry.Item angItem = telemetry.addData("angle", "%12.3f", 0.0);
        Telemetry.Item teltargetangle = telemetry.addData("target angle", "%12.3f", 0.0);
        telemetry.log().add("in turn");
        //get the current heading
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;
        float offset = 0;
        float targetangle;
        telemetry.log().add("result test: %f", heading - degrestoturn);
        //work out the target angle according to the direction of turn
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
        //use a offset to counter the issue of the imu going -180 to 180
        if (targetangle > 160) {
            offset = -160;
            targetangle += offset;
        } else if (targetangle < -160) {
            offset = 160;
            targetangle += offset;
        }
        heading += offset;
        int count = 0;

        //work out the difference between the target angle and the current heading and
        //turn until the difference is 0 + - the tolerance
        float diff = heading - targetangle;
        while (Math.abs(diff) > tolerance && count < 100000) {
            telemetry.log().add("count: %d", count);
            telemetry.log().add("offset: %f", offset);
            telemetry.log().add("diff: %f", diff);
            telemetry.log().add("heading: %f", heading);
            telemetry.log().add("target: %f", targetangle);
            telemetry.log().add("distance: %f", degrestoturn);
            if (direction == Turn.RIGHT) telemetry.log().add("turn: right");
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

    //function to calculate the ticks that corresponds to the distance in meters
    public static int CalcualteDistanceTicks(float distance) {
        //2941 ticks goes 1m
        return (int)(2941 * distance);
    }
    //set a defult power for if a power is not entered
    public static void driveForward( int distance) {
        driveForward(1, distance);
    }

    //if amount of ticks isn't entered calculat the amount of ticks and call the drive function
    public static void driveForward( float power, float distance) {
        driveForward(power, CalcualteDistanceTicks(distance));
    }

    //drive forward function
    public static void driveForward( float power, int distance) {


        boolean reachedmotor1 = false;
        boolean reachedmotor2 = false;

        //reset the encoders and motors
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(-power);


        int loopcount = 0;

        //continue to loop until the amount of ticks is reached by each motor
        while (reachedmotor1 == false || reachedmotor2 == false) {
            loopcount++;
            telemetry.log().add("in loop %d", loopcount);
            telemetry.log().add("current position 1 %d", robot.leftDrive.getCurrentPosition());
            telemetry.log().add("current position 2 %d", robot.rightDrive.getCurrentPosition());
            telemetry.log().add("reached motor 1 : %b", reachedmotor1);
            telemetry.log().add("reached motor 2 : %b", reachedmotor2);
            telemetry.log().add("distance %d", distance);

            //stop each motor independently and say that the motor is stopped when the distance is reached
            if (robot.leftDrive.getCurrentPosition() > distance) {
                telemetry.log().add("motor2 > distance");
                robot.leftDrive.setPower(0);
                reachedmotor1 = true;
            }
            if (robot.rightDrive.getCurrentPosition() > distance) {
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

    //calculate the amount of ticks if not entered
    public static void driveBackward( float power, float distance) {
        driveBackward(power, CalcualteDistanceTicks(distance));
    }

    //drive foward function
    public static void driveBackward( float power, int distance) {


        boolean reachedmotor1 = false;
        boolean reachedmotor2 = false;
        distance = -distance;
        //reet motors and encoders
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(power);


        int loopcount = 0;

        //run until each motor has reached the target distance
        while (reachedmotor1 == false || reachedmotor2 == false) {
            loopcount++;
            telemetry.log().add("in loop %d", loopcount);
            telemetry.log().add("current position 1 %d", robot.leftDrive.getCurrentPosition());
            telemetry.log().add("current position 2 %d", robot.rightDrive.getCurrentPosition());
            telemetry.log().add("reached motor 1 : %b", reachedmotor1);
            telemetry.log().add("reached motor 2 : %b", reachedmotor2);
            telemetry.log().add("distance %d", distance);

            //stop each motor independently and say that the motor is stopped when the distance is reached
            if (robot.leftDrive.getCurrentPosition() < distance) {
                telemetry.log().add("motor2 > distance");
                robot.leftDrive.setPower(0);
                reachedmotor1 = true;
            }
            if (robot.rightDrive.getCurrentPosition() < distance) {
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
    //jewel drop
    public static void Jewel( boolean toggle) {
        // lower the arm and wait a second
        robot.drop.setPosition(1);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if (robot.drop.getPosition() == 1 && (robot.Colour.blue() > 0 || robot.Colour.red() > 0)) {
            //check for which ball were looking for and change position accordingly
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
        //wait for the ball to be knocked of
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //reset arms back in there normal position
        robot.drop.setPosition(0.3);
        robot.push.setPosition(0.45);


    }

    //function to set the starting angle
    public static void setStartAngle(){
        Orientation angles;
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        startAngle = angles.firstAngle;
    }

    //function to fix the heading issue
    public static void fixHeading(){
        float heading;
        Orientation angles;
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;
        telemetry.log().add("heading%f" , heading);

        while (startAngle < heading){
            robot.leftDrive.setPower(-0.2);
            robot.rightDrive.setPower(0.2);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle;
        }
        while (startAngle > heading){
            robot.leftDrive.setPower(0.2);
            robot.rightDrive.setPower(-0.2);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle;
        }
    }

    //open the grabber
    public static void openGrabber(){
        while(!robot.max.isPressed()){
            robot.grabber.setPosition(0);
        }
        robot.grabber.setPosition(0.5);
    }

    //close the grabber
    public static void closeGrabber(){

        while(!robot.min.isPressed()){
            robot.grabber.setPosition(1);
        }
        robot.grabber.setPosition(0.5);
    }

    //move the y motion up a bit
    public static void moveyUp(){
        robot.ymotion.setPower(0.5);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.ymotion.setPower(0);
    }

    public static void moveservo(){
        robot.push.setPosition(0.45);
        robot.drop.setPosition(0.3);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}
