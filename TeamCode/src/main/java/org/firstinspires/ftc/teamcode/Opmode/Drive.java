/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Opmode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.util.MovingAverage;

@TeleOp(name = "Drive_", group = "Pushbot")

public class Drive extends LinearOpMode {

    /* Declare OpMode members. */
    private Configuration robot = new Configuration();   // Use a Pushbot's hardware
    private boolean toMax = false;
    private boolean toMin = false;
    private double multiplier = 0.8;
    private boolean lastButtonState = false;
    private boolean cat;


    @Override
    public void runOpMode() {


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        telemetry.setAutoClear(false);
        Telemetry.Item toggel = telemetry.addData("Toggel", "%12.3f", 0.0);


        DriveController.SetupTelemetry(telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            vec2 drive_bias = DriveController.GetDriveBias(gamepad1);

            toggel.setValue(multiplier);



            telemetry.update();

//            //Keep track of gamepad1.x which is just the x button
            boolean currentButtonState = gamepad1.x;
            //check if current button state is true and last button state is false
            //so that it will only because true when you realise the button
            if (currentButtonState && !lastButtonState) {
                if (multiplier == 0.8) {
                    multiplier = 0.2;
                } else {
                    multiplier = 0.8;
                }
            }
            //If current button states changed then change last button state
            if (currentButtonState != lastButtonState) {
                lastButtonState = currentButtonState;
            }

//            robot.leftDrive.setPower((drive_bias.x)*multiplier);
//            robot.rightDrive.setPower(  (-drive_bias.y)*multiplier);

            robot.leftDrive.setPower(((gamepad1.right_trigger - gamepad1.left_trigger)  + gamepad1.right_stick_x) * multiplier);
            robot.rightDrive.setPower(((gamepad1.right_trigger - gamepad1.left_trigger)   - gamepad1.right_stick_x) * multiplier);


            //lift
            if (gamepad2.dpad_up) {
                robot.ymotion.setPower(1);
            }
            if (gamepad2.dpad_down) {
                robot.ymotion.setPower(-1);
            }
            if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
                robot.ymotion.setPower(0);
            }
            robot.xmotion.setPower(gamepad2.left_stick_x * -1);

            //grabber
            if ((gamepad2.right_bumper && !gamepad2.left_bumper) && !toMin) {
                toMax = true;
            }
            if (toMax) {
                robot.grabber.setPosition(1);
                if (robot.min.isPressed()) {
                    robot.grabber.setPosition(0.5);
                    toMax = false;
                }
            }
            if ((gamepad2.left_bumper && !gamepad2.right_bumper) && !toMax) {
                toMin = true;
            }
            if (toMin) {
                robot.grabber.setPosition(0);
                if (robot.max.isPressed()) {
                    robot.grabber.setPosition(0.5);
                    toMin = false;
                }
            }


//            //extendor
            if (gamepad2.y) {
                robot.extentionUp.setPower(-1);
            }
            if (gamepad2.a) {
                robot.extentionUp.setPower(1);
            }
            if (!gamepad2.y && !gamepad2.a) {
                robot.extentionUp.setPower(0);
            }
            robot.extentionCross.setPower(gamepad2.right_stick_x * -1);
            if (gamepad1.dpad_left) {
                robot.picker.setPosition(0);
            }
            if (gamepad1.dpad_right) {
                robot.picker.setPosition(1);
            }
            if (!gamepad1.dpad_right && !gamepad1.dpad_left) {
                robot.picker.setPosition(0.5);
            }

        }
    }


}


