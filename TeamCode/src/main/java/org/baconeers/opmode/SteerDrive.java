package org.baconeers.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.baconeers.common.BaconOpMode;
import org.baconeers.common.ButtonControl;
import org.baconeers.common.GamePadDualMotorSteerDrive2;
import org.baconeers.common.GamePadLauncherSingle;
import org.baconeers.common.GamePadSafeDualMotor;
import org.baconeers.common.GamePadSafeDualMotorwinch;
import org.baconeers.common.GamePadSteerDrive;
import org.baconeers.common.GamePadToggleMotor;
import org.baconeers.common.GamePadToggleMotorWithRevers;
import org.baconeers.common.GamePadToggleServo;
import org.baconeers.common.KanaloaBallSorter;
import org.baconeers.common.WhileGamePadCRServo;
import org.baconeers.configurations.SmokeyBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "Relic Recovery ~ Smokey")
//@Disabled
public class SteerDrive extends BaconOpMode {

    private SmokeyBase robot;
    private GamePadSafeDualMotor winch;
    private GamePadSafeDualMotorwinch winch2;
    private GamePadToggleMotorWithRevers harvesterPrimary;
    private GamePadToggleMotorWithRevers harvesterSecondary;
    private WhileGamePadCRServo crServo;
    private Telemetry.Item avgItem;
    private Telemetry.Item maxItem;
    private KanaloaBallSorter kanaloaBallSorter;
    private GamePadToggleServo redServo;
    private GamePadSteerDrive drive;
    private GamePadLauncherSingle launcher;





    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {

        robot = SmokeyBase.newConfig(hardwareMap, telemetry);

        drive = new GamePadSteerDrive(this,gamepad1,robot.driveLeft,robot.driveRight);

        harvesterPrimary = new GamePadToggleMotorWithRevers(this,gamepad2,robot.harvesterPrimary, ButtonControl.A, ButtonControl.X,1.0f,false);
        harvesterSecondary = new GamePadToggleMotorWithRevers(this,gamepad2,robot.harvesterSecondary, ButtonControl.B, ButtonControl.Y,1.0f,false);

        launcher = new GamePadLauncherSingle(this,gamepad2,robot.launcher,ButtonControl.DPAD_LEFT,1.0f,false);

        avgItem = telemetry.addData("Avg", "%.3f ms", 0.0);
        avgItem.setRetained(true);

        maxItem = telemetry.addData("Max", "%.3f ms", 0.0);
        maxItem.setRetained(true);

    }

    /**
     * Implement this method to define the code to run when the Play button is pressed on the Driver station.
     * This code will run once.
     */
    @Override
    protected void onStart() throws InterruptedException {
        super.onStart();
//        kanaloaBallSorter.init();
    }

    /**
     * Implement this method to define the code to run when the Start button is pressed on the Driver station.
     * This method will be called in a loop on each hardware cycle
     */
    @Override
    protected void activeLoop() throws InterruptedException {

        // Update the Harvester
        harvesterPrimary.update();
        harvesterSecondary.update();

        drive.update();;
        launcher.update();

        movingAverageTimer.update();
        avgItem.setValue("%.3f ms", movingAverageTimer.movingAverage());
        maxItem.setValue("%.3f ms", movingAverageTimer.maxLoopTime());

    }

}
