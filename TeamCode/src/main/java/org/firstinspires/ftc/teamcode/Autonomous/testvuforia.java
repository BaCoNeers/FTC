package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.R;

@Autonomous(name = "Vuforia test")
public class testvuforia extends LinearOpMode {

    Telemetry.Item image_number;
    public CubeLocation location = CubeLocation.Unknown;

    public void runOpMode() throws InterruptedException {
        VuforiaController.VuforiaInit();
        //waits for the start button to be pressed


        image_number = telemetry.addData("image_number", "%12.0f", 0.0);

        waitForStart();
        //complete the run
        while (opModeIsActive()) {
            CubeLocation location = VuforiaController.VuforiaCheck();
        }

    }


}



