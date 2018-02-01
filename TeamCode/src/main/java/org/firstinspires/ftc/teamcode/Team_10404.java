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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Team_10404")

public class Team_10404 extends LinearOpMode {

    private DcMotor right = null;
    private DcMotor left = null;
    private DcMotor center = null;
    private DcMotor lift = null;
    private DcMotor claw = null;

    Servo side;



    private ElapsedTime runtime = new ElapsedTime();




    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        right = hardwareMap.dcMotor.get("Right");
        left = hardwareMap.dcMotor.get("Left");
        center = hardwareMap.dcMotor.get("Center");
        lift = hardwareMap.dcMotor.get("Lift");
        claw = hardwareMap.dcMotor.get("Claw");


        side = hardwareMap.servo.get("Side");

        right.setDirection(DcMotor.Direction.REVERSE);
        center.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            telemetry.update();

            right.setPower(gamepad1.right_stick_y);
            left.setPower(gamepad1.left_stick_y);

            if (gamepad1.right_bumper) {
                lift.setPower(.5);
            }
            else if (gamepad1.left_bumper){
                lift.setPower(-.5);
            }
            else {
                lift.setPower(0);
            }


            if (gamepad1.b){
                claw.setPower(.5);
            }
            else if(gamepad1.a){
                claw.setPower(-.5);
            }
            else{
                claw.setPower(0);
            }

            if (gamepad1.x){
                side.setPosition(.5);
            }

            if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0){
                center.setPower(1);
            }else if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0){
                center.setPower(-1);
            }else {
                center.setPower(0);
            }
            /*

            while (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0) {
                center.setPower(1);
            }
            while (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0)   {
                center.setPower(-1);
            }
            */
        }

    }
}
