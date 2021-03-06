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

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "simons")

public class  Simons_New_Autonomous extends LinearOpMode {

    // Declare OpMode members.
    private Configuration robot = new Configuration();

    //declare variables
    private double ppr;
    private double degree;

    Telemetry.Item left_encoder = telemetry.addData("left encoder", "%12.3f", 0.0);
    Telemetry.Item right_encoder = telemetry.addData("right encoder", "%12.3f", 0.0);


    @Override
    public void runOpMode() {

        telemetry.setAutoClear(false);

        robot = new Configuration();
        //initializes the configuration
        robot.init(hardwareMap);

        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        robot.leftDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.rightDrive.setDirection(DcMotor.Direction.FORWARD);

        //sets the configuration of the robot


        setup_encoders();
        encoders();
        wheel_distance(2);
        turn_distance(7);
        waitForStart();

        turn(180);
        drive(1);
        //complete the run
        if (opModeIsActive()) {

        }
    }


    private void encoders(){
        int encoder_tick_right = robot.rightDrive.getCurrentPosition();
        int encoder_tick_left = robot.leftDrive.getCurrentPosition();
        left_encoder.setValue(encoder_tick_left);
        right_encoder.setValue(encoder_tick_right);
        telemetry.update();
    }

    private void setup_encoders(){
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void wheel_distance(double radius){
        double circumference = radius*Math.PI;
        ppr = circumference/1120;
    }

    private void turn_distance(double radius){
        double circumference = radius*Math.PI;
        degree = circumference/360;
    }

    private void drive(int distance){
        distance = (int)Math.round(distance/ppr);
        setup_encoders();
        robot.leftDrive.setTargetPosition(distance);
        robot.rightDrive.setTargetPosition(distance);
        if(distance>0){
            robot.leftDrive.setPower(0.5);
            robot.rightDrive.setPower(0.5);
        }
        else if(distance<0){
            robot.rightDrive.setPower(-0.5);
            robot.leftDrive.setPower(-0.5);
        }
        else{
            robot.rightDrive.setPower(0);
            robot.leftDrive.setPower(0);
        }
        while(opModeIsActive() && (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())){
            encoders();
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void turn(int degrees){
        if (opModeIsActive()) {
            double distance = degree * degrees;
            int required_ppr = (int)Math.round(distance/ppr);
            setup_encoders();
            robot.leftDrive.setTargetPosition(required_ppr);
            robot.rightDrive.setTargetPosition(-required_ppr);
            if(degrees>0){
                robot.leftDrive.setPower(0.3);
                robot.rightDrive.setPower(-0.3);
            }
            else if(degrees<0){
                robot.leftDrive.setPower(-0.3);
                robot.rightDrive.setPower(0.3);
            }
            else{
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
            }

            while(opModeIsActive() && (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())){
                encoders();
            }
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}

