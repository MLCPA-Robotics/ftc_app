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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Team_10404_Auton_AXx  ")

public class Team_10404_Auton extends LinearOpMode {

    private DcMotor right = null;
    private DcMotor left = null;
    private DcMotor center = null;
    private DcMotor lift = null;
    private DcMotor Claw = null;
    Servo side;

     public void forward_back(double power, int duration){
         right.setPower(power);
         left.setPower(power);
         sleep(duration);
     }

     public void move(double power, String turn, int duration){
         // for turn, 1 = right turn, -1 = left turn, 0 = straight

         if (turn.equals("right")){
             right.setPower(0);
             left.setPower(power);
         }
         else if (turn.equals("left")){
             right.setPower(power);
             left.setPower(0);
         }
         else if(turn.equals("straight")) {
             right.setPower(power);
             left.setPower(power);
         }
         sleep(duration);
     }

    public void stopRobot(){
        left.setPower(0);
        right.setPower(0);
        sleep(1);
    }

    public void Move(String turn, int duration){
        if(turn.equals("straight")){
            left.setPower(.3);
            right.setPower(.3);
        }
        else if(turn.equals("left")){
            left.setPower(0);
            right.setPower(.3);
        }
        else if(turn.equals("right")){
            left.setPower(.3);
            right.setPower(0);
        }
        sleep(duration);
        stopRobot();
    }
    public void Backwards(int duration){
        left.setPower(-.25);
        right.setPower(-.25);
        sleep(duration);
        stopRobot();
    }



    private ElapsedTime runtime = new ElapsedTime();




    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        right = hardwareMap.dcMotor.get("Right");
        left = hardwareMap.dcMotor.get("Left");
        center = hardwareMap.dcMotor.get("Center");
        lift = hardwareMap.dcMotor.get("Lift");
        Claw = hardwareMap.dcMotor.get("Claw");

        side = hardwareMap.servo.get("Side");

        left.setDirection(DcMotor.Direction.REVERSE);
        center.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            telemetry.update();

            move(.2,"straight",2000);

            Claw.setPower(.2);
            left.setPower(-.2);
            right.setPower(-.2);
            sleep(1000);
            Claw.setPower(0);
            stopRobot();

            Move("straight", 350);



            right.setPower(0);
            left.setPower(0);
            sleep(40000000);

        }

    }
}
