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
        VuforiaInit();
        //initializes the motors
        RobotInit();
        //waits for the start button to be pressed
        waitForStart();
        //complete the run
        Run2();

    }

    public VuforiaTrackable Trackables[]=new VuforiaTrackable[3];
    //initializes Vuforia with the parameters set
    public void VuforiaInit() {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        params.vuforiaLicenseKey = "AQQN8vT/////AAAAGR2kWcTZSEaGsNmfOFgjVCpfRMD0rrC8iVwL5YiD9FQny/LDfDTPHuZMkS31CZvPgOpu9GPC10zAHbs2om9lY3IZmlQ944EDdEeCFkzTFlN5Fk1/gzwUbMgR1+8qwBy/7FsoQOgXFApTWMRfogt6FqXahm7g0gpfzDiOhAPHgHmMDYL5wqHdBgRdt12rT6FnwePm7H3Z7hcEPh7BwLoD8wFa9mqhDnNkm2czsZLiGgQQGy3bdWY3kq3Hzn6XNDREjq4xk2RmTMWZi6BFDZgFAMaaTT2PdLoF6waMR+o21FW/EHCRd1fJu1fNPSvtyLdwxkUG+JrjVtTBBQGrQ5mHRuZ/Bp0XlHijhW0KEh6/G7lb";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables cyper = vuforia.loadTrackablesFromAsset("RelicVuMark");
        for (int i=0; i< cyper.size();i++) {
            Trackables[i] = cyper.get(i);
            Trackables[i].setName(Integer.toString(i));
        }
        cyper.activate();

    }

    public void VuforiaCheck() {
        for (int n=0; n < Trackables.length; n++){
            try {
                if (((VuforiaTrackableDefaultListener) Trackables[n].getListener()).isVisible()) {
                    telemetry.log().add("Vuforia", Trackables[n].getName());
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
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

    public void Run8() {
        //True:blue False:red
        RobotController.setStartAngle();
        RobotController.moveyUp();
        RobotController.Jewel(false);
       // RobotController.driveForward(0.3f, 0.1f);
        RobotController.driveBackward(0.3f , 0.6f);
        RobotController.fixHeading();
        RobotController.driveBackward(0.3f,0.55f);
        RobotController.turn(90, Turn.RIGHT);
        RobotController.driveForward(0.3f, 0.2f);
        RobotController.openGrabber();
        RobotController.driveBackward(0.3f,0.55f);
    }

    public void Run4(){

        RobotController.setStartAngle();
        RobotController.moveyUp();
        RobotController.Jewel(true);
        RobotController.driveForward(0.3f , 0.3f);
        //RobotController.fixHeading();
       VuforiaCheck();
       RobotController.driveForward(0.3f, 0.5f);
        RobotController.turn(90, Turn.RIGHT);
        RobotController.driveForward(0.3f, 0.1f);
        RobotController.openGrabber();
        RobotController.driveBackward(0.3f,0.25f);
        RobotController.driveForward(0.3f,0.2f);
    }
    public void Run2(){

        RobotController.setStartAngle();
        RobotController.moveyUp();
        RobotController.Jewel(false);
        RobotController.driveBackward(0.3f , 0.3f);
        RobotController.fixHeading();
        //VuforiaCheck();
        RobotController.driveBackward(0.3f, 0.5f);
        RobotController.turn(90, Turn.RIGHT);
        RobotController.driveForward(0.3f, 0.3f);
        RobotController.openGrabber();
        RobotController.driveBackward(0.3f,0.25f);
        RobotController.driveForward(0.3f,0.25f);
    }

    public void testImu(){
        RobotController.turn(90, Turn.RIGHT);
    }




}



