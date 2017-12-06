package org.firstinspires.ftc.teamcode.Opmode;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by me on 6/12/2017.
 */

public class DriveController {

    public static vec2 GetDriveBias(Gamepad gamepad) {

        // Retrieve the power
        double drive_power = gamepad.left_stick_y;
        double turning_power = gamepad.right_stick_x;

        return GetDriveBias(drive_power, turning_power);
    }

    public static vec2 GetDriveBias(double drive_power, double turning_power) {

        // Bias Settings
        vec2 clockwise_bias = new vec2(-1,1);
        vec2 forward_bias = new vec2(1,1);

        vec2 result = vec2.lerp(forward_bias.multiply(1.0 - Math.abs(turning_power)), clockwise_bias, turning_power).normalize().multiply(drive_power);

        return result;
    }
}
