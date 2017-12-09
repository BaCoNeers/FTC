package org.firstinspires.ftc.teamcode.Autonomous;

import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

/**
 * Created by me on 10/12/2017.
 */

public class VuforiaController {

    private static CubeLocation location = CubeLocation.Unknown;
    public static VuforiaTrackable Trackables[] = new VuforiaTrackable[3];

    //initializes Vuforia with the parameters set
    public static void VuforiaInit() {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        params.vuforiaLicenseKey = "AQQN8vT/////AAAAGR2kWcTZSEaGsNmfOFgjVCpfRMD0rrC8iVwL5YiD9FQny/LDfDTPHuZMkS31CZvPgOpu9GPC10zAHbs2om9lY3IZmlQ944EDdEeCFkzTFlN5Fk1/gzwUbMgR1+8qwBy/7FsoQOgXFApTWMRfogt6FqXahm7g0gpfzDiOhAPHgHmMDYL5wqHdBgRdt12rT6FnwePm7H3Z7hcEPh7BwLoD8wFa9mqhDnNkm2czsZLiGgQQGy3bdWY3kq3Hzn6XNDREjq4xk2RmTMWZi6BFDZgFAMaaTT2PdLoF6waMR+o21FW/EHCRd1fJu1fNPSvtyLdwxkUG+JrjVtTBBQGrQ5mHRuZ/Bp0XlHijhW0KEh6/G7lb";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables cyper = vuforia.loadTrackablesFromAsset("RelicVuMark");
        for (int i = 0; i < cyper.size(); i++) {
            Trackables[i] = cyper.get(i);
            Trackables[i].setName(Integer.toString(i));
        }
        cyper.activate();




    }

    public static CubeLocation VuforiaCheck() {

        for (int n = 0; n < Trackables.length; n++) {
            try {
                if (((VuforiaTrackableDefaultListener) Trackables[n].getListener()).isVisible()) {

                    switch (n) {
                        case 0:
                            location = CubeLocation.Left;
                            break;
                        case 1:
                            location = CubeLocation.Center;
                            break;
                        case 2:
                            location = CubeLocation.Right;
                            break;
                        default:
                            location = CubeLocation.Unknown;
                    }

                }
            } catch (Exception e) {
                location = CubeLocation.Unknown;
            }
        }

        return location;
    }
}
