package org.firstinspires.ftc.teamcode.Opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
        */



@TeleOp(name="drive", group="Pushbot")

public class vec2 {

    public double x = 0;
    public double y = 0;

    public vec2() {

    }

    public vec2(double _x, double _y) {
        x = _x;
        y = _y;
    }

    public static vec2 lerp(vec2 a, vec2 b, double amount) {
        vec2 b_take_a = b.subtract(a);
        vec2 b_mul_a = b_take_a.multiply(amount);
        vec2 RESULT = a.add(b_mul_a);
        return RESULT;
    }

    public vec2 add(vec2 value) {
        return new vec2(x +value.x, y + value.y );
    }

    public vec2 subtract(vec2 value) {
        return new vec2(x -value.x, y - value.y );
    }

    public vec2 multiply(vec2 value) {
        return new vec2(x *value.x, y * value.y );
    }

    public vec2 divide(vec2 value) {
        return new vec2(x /value.x, y / value.y );
    }

    public vec2 add(double value) {
        return new vec2(x +value, y + value );
    }

    public vec2 subtract(double value) {
        return new vec2(x -value, y - value );
    }

    public vec2 multiply(double value) {
        return new vec2(x *value, y * value );
    }

    public vec2 divide(double value) {
        return new vec2(x /value, y / value );
    }

    public vec2 normalize() {
        return this.divide(this.length());
    }

    public double length() {
        return Math.sqrt(length2());
    }

    public double length2() {
        return x*x+y*y;
    }


}
