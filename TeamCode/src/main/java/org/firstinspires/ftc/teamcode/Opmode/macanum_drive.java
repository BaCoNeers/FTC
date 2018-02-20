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

@TeleOp(name = "macanum", group = "Pushbot")

public class macanum_drive extends LinearOpMode {

    /* Declare OpMode members. */
    private Configuration robot = new Configuration();   // Use a Pushbot's hardware
    private double x;
    private double y;
    private double circlex;
    private double circley;
    private double radius;
    private double circumfrance;
    private double angle;
    private double power;
    private double add = 1-((1/8)*Math.PI);
    private double motor_power_1;
    private double motor_power_2;
    private double motor_power_3;
    private double motor_power_4;
    private boolean check = false;


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



        DriveController.SetupTelemetry(telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.update();

            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            check = false;

            circlex = x*Math.sqrt(1-(Math.pow(y,2)/2));
            circley = y*Math.sqrt(1-(Math.pow(x,2)/2));

            radius = Math.sqrt(Math.pow(circlex,2)+Math.pow(circley,2));

            circumfrance = (2*radius*Math.PI)/360;


            if(y>0){
                if(x>0){
                    if(x*20<y){
                        motor_power_1 = radius;
                        motor_power_2 = radius;
                        motor_power_3 = radius;
                        motor_power_4 = radius;
                    }
                    else if((x*1.1) < y || y < (x/1.1)){
                        motor_power_1 = radius;
                        motor_power_2 = 0;
                        motor_power_3 = radius;
                        motor_power_4 = 0;
                    }
                    else if(x/20>y){
                        motor_power_1 = radius;
                        motor_power_2 = -radius;
                        motor_power_3 = -radius;
                        motor_power_4 = radius;
                    }
                    else if(x>y){
                        angle = Math.acos(y/radius);
                        power = (angle*(circumfrance/360))+add;
                        motor_power_1 = power+((circumfrance/8)-power);
                        motor_power_2 = power;
                        motor_power_3 = power+((circumfrance/8)-power);
                        motor_power_4 = power;
                    }
                    else{
                        angle = Math.acos(x/radius);
                        power = (angle*(circumfrance/360))+add;
                        motor_power_1 = power+((circumfrance/8)-power);
                        motor_power_2 = -power;
                        motor_power_3 = -power+((circumfrance/8)-power);
                        motor_power_4 = power;
                    }
                }
                else{
                    if(-x*20>y){
                        motor_power_1 = -radius;
                        motor_power_2 = -radius;
                        motor_power_3 = -radius;
                        motor_power_4 = -radius;
                    }
                    else if(-x*1.1>y || -x/1.1<y ){
                        motor_power_1 = 0;
                        motor_power_2 = -radius;
                        motor_power_3 = -radius;
                        motor_power_4 = 0;
                    }
                    else if(-x/20<y){
                        motor_power_1 = radius;
                        motor_power_2 = -radius;
                        motor_power_3 = -radius;
                        motor_power_4 = radius;
                    }
                    else if(x*-1>y){
                        angle = Math.acos(x*-1/radius);
                        power = (angle*(circumfrance/360))+add;
                        motor_power_1 = power;
                        motor_power_2 = -power-((circumfrance/8)-power);;
                        motor_power_3 = -power-((circumfrance/8)-power);
                        motor_power_4 = power;
                    }
                    else{
                        angle = Math.acos(y/radius);
                        power = (angle*(circumfrance/360))+add;
                        motor_power_1 = -power;
                        motor_power_2 = -power-((circumfrance/8)-power);;
                        motor_power_3 = -power-((circumfrance/8)-power);
                        motor_power_4 = -power;
                    }
                }
            }
            else{
                if(x>0){
                    if(-x*20>y){
                        motor_power_1 = radius;
                        motor_power_2 = radius;
                        motor_power_3 = radius;
                        motor_power_4 = radius;
                    }
                    if(-x*1.1>y || -x/1.1<y ){
                        motor_power_1 = 0;
                        motor_power_2 = radius;
                        motor_power_3 = radius;
                        motor_power_4 = 0;
                    }
                    if(-x/20<y){
                        motor_power_1 = -radius;
                        motor_power_2 = radius;
                        motor_power_3 = radius;
                        motor_power_4 = -radius;
                    }
                    if(x>y*-1){
                        angle = Math.acos(y*-1/radius);
                        power = (angle*(circumfrance/360))+add;
                        motor_power_1 = power;
                        motor_power_2 = power+((circumfrance/8)-power);;
                        motor_power_3 = power+((circumfrance/8)-power);
                        motor_power_4 = power;
                    }
                    else{
                        angle = Math.acos(x/radius);
                        power = (angle*(circumfrance/360))+add;
                        motor_power_1 = -power;
                        motor_power_2 = power+((circumfrance/8)-power);;
                        motor_power_3 = power+((circumfrance/8)-power);
                        motor_power_4 = -power;
                    }
                }
                else{
                    if(x*20<y){
                        motor_power_1 = -radius;
                        motor_power_2 = -radius;
                        motor_power_3 = -radius;
                        motor_power_4 = -radius;
                    }
                    if(x/20>y){
                        motor_power_1 = -radius;
                        motor_power_2 = radius;
                        motor_power_3 = radius;
                        motor_power_4 = -radius;
                    }
                    if((x*1.1) < y || y < (x/1.1)){
                        motor_power_1 = 0;
                        motor_power_2 = -radius;
                        motor_power_3 = -radius;
                        motor_power_4 = 0;
                    }
                    if(x*-1>y*-1){
                        angle = Math.acos(y*-1/radius);
                        power = (angle*(circumfrance/360))+add;
                        motor_power_1 = -power;
                        motor_power_2 = power-((circumfrance/8)-power);;
                        motor_power_3 = power-((circumfrance/8)-power);
                        motor_power_4 = -power;
                    }
                    else{
                        angle = Math.acos(x*-1/radius);
                        power = (angle*(circumfrance/360))+add;
                        motor_power_1 = -power;
                        motor_power_2 = -power-((circumfrance/8)-power);;
                        motor_power_3 = -power-((circumfrance/8)-power);
                        motor_power_4 = -power;
                    }
                }
            }



        }
    }


}


