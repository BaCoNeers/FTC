package org.baconeers.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Operation to assist with Gamepad actions on DCMotors
 */
public class GamePadLauncherSingle extends BaconComponent {

    private final ButtonControl buttonControl;
    private final ButtonControl buttonControl2;
    private final DcMotor motor;
    private final Gamepad gamepad;
    private final TouchSensor touchSensorLauncher;
    private float motorPower;
    private boolean motorOn = false;
    private boolean reverseOn = false;
    private boolean lastButtonState = false;
    private final Telemetry.Item item;
    private double lastpower;
    private boolean showtelemetry = false;
    private double shotCount = 0;
    private boolean hasRemovedShot = true;


    /**
     * Constructor for operation.  Telemetry enabled by default.
     *
     * @param opMode
     * @param gamepad       Gamepad
     * @param motor         DcMotor to operate on
     * @param buttonControl {@link ButtonControl}
     * @param power         power to apply when using gamepad buttons
     * @param showTelemetry  display the power values on the telemetry
     */
    public GamePadLauncherSingle(BaconOpMode opMode, Gamepad gamepad, DcMotor motor, ButtonControl buttonControl, ButtonControl buttonControl2, float power, boolean showTelemetry) {
        super(opMode);

        this.gamepad = gamepad;
        this.motor = motor;
        this.buttonControl = buttonControl;
        this.buttonControl2 = buttonControl2;
        this.motorPower = power;
        this.touchSensorLauncher = touchSensorLauncher;

        if (showTelemetry) {
            item = opMode.telemetry.addData("Control " + buttonControl.name(), 0.0f);
            item.setRetained(true);
        } else {
            item = null;
        }
    }
    public GamePadLauncherSingle(BaconOpMode opMode, Gamepad gamepad, DcMotor motor, ButtonControl buttonControl, ButtonControl buttonControl2, float power) {
        this(opMode,gamepad,motor,buttonControl,buttonControl2,power,true);
    }


    /**
     * Update motors with latest gamepad state
     */
    public void update() {

        boolean pressed = buttonPressed(gamepad, buttonControl);
        boolean pressed2 = buttonPressed(gamepad,buttonControl2);

        // if you press the button, add a shot to the queue
        if (!pressed) {
            shotCount++;
        }

        //if there are shots in the queue
        if (shotCount > 0) {
            //if the launcher has completed a revolution and touched the button, remove a shot from the queue
            if (pressed && !hasRemovedShot) {
                --shotCount;
                hasRemovedShot = true;
            }

            // if the button is no longer being pressed, then allow a shot to be removed
            if (!pressed) {
                hasRemovedShot = false;
            }

            //add power
            motor.setPower(0.85);

            //if there are no shots in the queue, allow the user to manually shoot
        } else if (gamepad2.B == ButtonState.ACTIVE && !gamepad2.getGamepad().start && !motorLauncher.isBusy()) {
            motor.setPower(0.85);
        } else { // if nothing is happening then stop
            motor.setPower(0);
        }

        /* John's Code:
        if (ENABLE_LAUNCHER) {

            // if you press y add a shot to the queue
            if (gamepad2.Y == ButtonState.RELEASED) {
                shotCount++;
            }

            // if there are shots in the queue
            if (shotCount > 0) {

                //if the launcher has completed a revolution and touched the button, remove a shot from the queue
                if (touchSensorLauncher.isPressed() && !hasRemovedShot) {
                    --shotCount;
                    hasRemovedShot = true;
                }
                // if the button is no longer being pressed, then allow a shot to be removed
                if (!touchSensorLauncher.isPressed()) {
                    hasRemovedShot = false;
                }

                //add power
                motorLauncher.setPower(0.85);

                //if there are no shots in the queue, allow the user to manually shoot
            } else if (gamepad2.B == ButtonState.ACTIVE && !gamepad2.getGamepad().start && !motorLauncher.isBusy()) {
                motorLauncher.setPower(0.85);
            } else { // if nothing is happening then stop
                motorLauncher.stop();
            }
        }*/

        getOpMode().telemetry.log().add("%s motor power: %.2f", pressed, motor.getPower(), motorPower);
        getOpMode().telemetry.log().add("%s motor power: %.2f", pressed2, motor.getPower(),motorPower);

    }


}

