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
import org.firstinspires.ftc.teamcode.util.MovingAverageTimer;

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

public class Drive extends LinearOpMode {

    /* Declare OpMode members. */
    private Configuration robot           = new Configuration();   // Use a Pushbot's hardware

    //tells me if either left trigger or right trigger as been pressed
    private boolean toMax = false;
    private boolean toMin = false;

    //Keep power for each motor
    private double leftPower;
    private double rightPower;

    //Allows reduce power
    private double divider;

    //Keep the last button state for a toggel
    private boolean lastButtonState = false;

    //Creating instances
    private MovingAverage leftAvarage = new MovingAverage(8);
    private MovingAverage rightAvarage = new MovingAverage(8);
    private MovingAverageTimer avg = new MovingAverageTimer();
    private JewelDrop jewel = new JewelDrop();

    public double lurp(double a,double b,double z){
        double number = 0;
        number = a+(b-a)*z;
        return number;
    }

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
        Telemetry.Item avgItem = telemetry.addData("average" , "%12.3f", 0.0);
        Telemetry.Item DriveLeft = telemetry.addData("Left drive" , "%12.3f", 0.0);
        Telemetry.Item DriveRight = telemetry.addData("Right drive" , "%12.3f", 0.0);
        Telemetry.Item DriveLeftAvg = telemetry.addData("Left drive avarage" , "%12.3f", 0.0);
        Telemetry.Item DriveRightAvg = telemetry.addData("Right drive avarage" , "%12.3f", 0.0);

        divider = 1.0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            avg.update();

            avgItem.setValue("%12.3f",avg.average());
            DriveLeft.setValue("%12.3f",leftPower);
            DriveRight.setValue("%12.3f",rightPower);
            DriveLeftAvg.setValue("%12.3f",leftAvarage.getMovingAverage());
            DriveRightAvg.setValue("%12.3f",rightAvarage.getMovingAverage());

            telemetry.update();


            //Drive
            //Scale power is forward movement
            double scalePower = (gamepad1.right_stick_x*-1);
            double steer = (gamepad1.left_stick_y*-1);
            //If forward movement (scalePower) has no number, it will just use steer to move
            if (scalePower == 0.0f) {
                leftPower = steer;
                rightPower = -steer;
            }
            //else
            else {
                leftPower = scalePower * ((steer < 0) ? 1.0f + steer : 1.0f);
                rightPower = scalePower * ((steer > 0) ? 1.0f - steer : 1.0f);
            }

            boolean currentButtonState = gamepad1.x;

            if (currentButtonState && !lastButtonState) {
                if (divider == 1.0) {
                    divider = 0.3;
                }
                else {
                    divider = 1.0;
                }
            }

            if(currentButtonState != lastButtonState){
                lastButtonState = currentButtonState;
            }

            robot.leftDrive.setPower(leftAvarage.add(leftPower*divider));
            robot.rightDrive.setPower(rightAvarage.add(rightPower*divider));



//            if (gamepad1.left_stick_y>0){
//                leftStickY = true;
//            }
//            else{
//                leftStickY = false;
//            }
//            if(gamepad1.right_stick_x>0){
//                rightStickX = true;
//            }
//            else{
//                rightStickX = false;
//            }
//            Forward = Math.max(Math.pow(gamepad1.left_stick_y,2),oldForward);
//            Turning = Math.max(Math.pow(gamepad1.right_stick_x,2),oldTurning);
//
//            robot.rightDrive.setPower(Forward+Turning);
//            robot.leftDrive.setPower(Forward-Turning);





            //lift
            //Just setting the motors of the lift
            robot.ymotion.setPower(gamepad2.right_stick_x * -1);
            robot.xmotion.setPower(gamepad2.right_stick_y * -1);

            //grabber
            //This will first set toMax or toMix to true when the bumper is pressed
            //So that i know it has been pressed, once its pressed i will move the grabber and
            //when the opposite touch sensor is pressed it will stop moving the servo and the
            //booleans (toMax and toMin) will be set to false to stop it moving.
            if(gamepad2.left_bumper){
                toMax = true;
            }
            if (toMax){
                robot.grabber.setPosition(1);
                if(robot.min.isPressed()){
                    robot.grabber.setPosition(0.5);
                    toMax = false;
                }
            }
            if(gamepad2.right_bumper){
                toMin = true;
            }
            if(toMin){
                robot.grabber.setPosition(0);
                if(robot.max.isPressed()){
                    robot.grabber.setPosition(0.5);
                    toMin = false;
                }
            }



            //extendor
            //Setting lift motor to x and y stick
           robot.extentionUp.setPower(gamepad2.left_stick_y);
           robot.extentionCross.setPower(gamepad2.left_stick_x);
           //THis will move the servo if dpad is pressed other wise it will be set to 0.5
            if (gamepad1.dpad_left){
                robot.picker.setPosition(0);
            }
            if(gamepad1.dpad_right){
                robot.picker.setPosition(1);
            }
            if(!gamepad1.dpad_right && !gamepad2.dpad_left){
                robot.picker.setPosition(0.5);
            }


        }
    }


}


