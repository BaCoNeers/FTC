package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
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

        robot = new Configuration();

        robot.init(hardwareMap);

        VuforiaInit();

        RobotInit();

        waitForStart();

        Run1();

    }

    public void VuforiaInit() {

        // Vuforia image recognition
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        params.vuforiaLicenseKey = "AQQN8vT/////AAAAGR2kWcTZSEaGsNmfOFgjVCpfRMD0rrC8iVwL5YiD9FQny/LDfDTPHuZMkS31CZvPgOpu9GPC10zAHbs2om9lY3IZmlQ944EDdEeCFkzTFlN5Fk1/gzwUbMgR1+8qwBy/7FsoQOgXFApTWMRfogt6FqXahm7g0gpfzDiOhAPHgHmMDYL5wqHdBgRdt12rT6FnwePm7H3Z7hcEPh7BwLoD8wFa9mqhDnNkm2czsZLiGgQQGy3bdWY3kq3Hzn6XNDREjq4xk2RmTMWZi6BFDZgFAMaaTT2PdLoF6waMR+o21FW/EHCRd1fJu1fNPSvtyLdwxkUG+JrjVtTBBQGrQ5mHRuZ/Bp0XlHijhW0KEh6/G7lb";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables cyper = vuforia.loadTrackablesFromAsset("RelicVuMark");
        for (int i=0; i< cyper.size();i++) {
            cyper.get(i).setName(Integer.toString(i));
        }
        cyper.activate();
    }

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
    }

    public void Run1() {
        //True:blue False:red
        RobotController.Jewel(false);
        RobotController.driveForward(-0.3f, -0.38f);
        RobotController.driveForward(0.3f , 1f);
        RobotController.driveForward(0.3f,0.55f);
        RobotController.turn(90, Turn.LEFT);
        RobotController.driveForward(0.3f, 0.2f);
    }









}



