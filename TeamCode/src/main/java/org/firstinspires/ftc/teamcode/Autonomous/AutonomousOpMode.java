package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.R;

/**
 * Autonomous program being used for test
 */

@Autonomous(name = "Autonomous")
public class AutonomousOpMode extends LinearOpMode {
    // Declare drive motors
    Configuration robot;

    /*
    Autonomous entry point
     */
    @Override
    public void runOpMode() throws InterruptedException {
        //sets the configuration of the robot
        robot = new Configuration();
        //initializes the configuration
        robot.init(hardwareMap);
        //calls the Vuforia initialization
        //VuforiaInit();
        //initializes the motors
        RobotInit();
        robot.push.setPosition(0.4);
        robot.drop.setPosition(0);
        //waits for the start button to be pressed

        VuforiaController.VuforiaInit();

        waitForStart();
        //complete the run
        if (opModeIsActive()) {
            blue1();
        }

    }


    //initilizes robots motors and resets the grabber
    public void RobotInit() {
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

        RobotController.RegisterAutonomousController(this);
        RobotController.closeGrabber();
    }



    public void blue1() {

        RobotController.setStartAngle();
        RobotController.moveyUp();
        RobotController.Jewel(true);
        RobotController.driveForward(0.3f, 0.40f);
        RobotController.turn(45, Turn.RIGHT);
        RobotController.driveForward(0.3f, 0.3f);
        RobotController.openGrabber();
        RobotController.driveBackward(0.3f, 0.1f);
    }

    public void blue2(){
        RobotController.setStartAngle();
        RobotController.moveyUp();
        RobotController.Jewel(true);
        RobotController.driveForward(0.3f, 0.40f);
        RobotController.turn(45, Turn.LEFT);
        RobotController.driveForward(0.3f, 0.2f);
        RobotController.openGrabber();
        RobotController.driveBackward(0.3f, 0.1f);
    }

    public void red1() {
        RobotController.setStartAngle();
        RobotController.moveyUp();
        RobotController.Jewel(false);
        RobotController.driveBackward(0.3f, 0.40f);
        RobotController.turn(90, Turn.RIGHT);
        RobotController.turn(45, Turn.RIGHT);
        RobotController.driveForward(0.3f, 0.3f);
        RobotController.openGrabber();
        RobotController.driveBackward(0.3f, 0.1f);
    }

    public void red2(){
        RobotController.setStartAngle();
        RobotController.moveyUp();
        RobotController.Jewel(false);
        RobotController.driveForward(0.3f, 0.40f);
        RobotController.turn(90,Turn.LEFT);
        RobotController.turn(45, Turn.LEFT);
        RobotController.driveForward(0.3f, 0.2f);
        RobotController.openGrabber();
        RobotController.driveBackward(0.3f, 0.1f);
    }

}



