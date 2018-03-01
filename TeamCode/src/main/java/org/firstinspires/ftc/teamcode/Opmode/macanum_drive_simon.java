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
import org.firstinspires.ftc.teamcode.util.Configuration_OBot;

@TeleOp(name = "macanum", group = "Pushbot")

public class macanum_drive_simon extends LinearOpMode {

    /* Declare OpMode members. */
    private Configuration_OBot robot = new Configuration_OBot();
    private double x;
    private double y;
    private double DS;
    private double angle;
    private double turn;
    private double motor_power_1;
    private double motor_power_2;
    private double motor_power_3;
    private double motor_power_4;


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
        Telemetry.Item left_stick_y = telemetry.addData("y: ", "%12.3f", 0.0);
        Telemetry.Item left_stick_x = telemetry.addData("x: ", "%12.3f", 0.0);

        DriveController.SetupTelemetry(telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.update();

            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y*-1;

            DS = Math.sqrt(Math.pow(x,2)+Math.pow(y,2));
            angle = Math.atan(x/y);
            turn = gamepad1.right_stick_x;

            robot.DriveFL.setPower(DS*Math.cos(angle+(4/Math.PI))-turn);
            robot.DriveFR.setPower(DS*Math.sin(angle+(4/Math.PI))+turn);
            robot.DriveBL.setPower(DS*Math.sin(angle+(4/Math.PI))-turn);
            robot.DriveBR.setPower(DS*Math.cos(angle+(4/Math.PI))+turn);


            left_stick_x.setValue(x);
            left_stick_y.setValue(y);
            telemetry.update();


        }
    }


}


