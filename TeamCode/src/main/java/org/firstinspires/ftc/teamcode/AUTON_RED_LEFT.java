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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Vumark_RED_LEFT_JEWEL'", group ="Concept")
@Disabled
public class AUTON_RED_LEFT extends LinearOpMode {

    private DcMotor right = null;
    private DcMotor left = null;
    private DcMotor lift = null;

    private DcMotor open_close = null;
    private Servo Side;


    private ColorSensor Color_sensor;
    private DeviceInterfaceModule cdim;
    static final int LED_CHANNEL = 5;


    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        right = hardwareMap.dcMotor.get("Right");
        left = hardwareMap.dcMotor.get("Left");
        lift = hardwareMap.dcMotor.get("Lift");

        open_close = hardwareMap.dcMotor.get("Open_close");
        Side = hardwareMap.servo.get("Side");


        right.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);



        float hsvValues[] = {0F,0F,0F};

        final float values[] = hsvValues;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        boolean bPrevState = false;
        boolean bCurrState = false;


        boolean bLedOn = true;

        cdim = hardwareMap.deviceInterfaceModule.get("dim");

        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannel.Mode.OUTPUT);

        Color_sensor = hardwareMap.colorSensor.get("sensor_color");

        cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);

        waitForStart();

        while (opModeIsActive()) {

            bCurrState = gamepad1.x;

            if ((bCurrState == true) && (bCurrState != bPrevState))  {

                bLedOn = !bLedOn;
                cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);
            }

            bPrevState = bCurrState;

            Color.RGBToHSV((Color_sensor.red() * 255) / 800, (Color_sensor.green() * 255) / 800, (Color_sensor.blue() * 255) / 800, hsvValues);

            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", Color_sensor.alpha());
            telemetry.addData("Red  ", Color_sensor.red());
            telemetry.addData("Green", Color_sensor.green());
            telemetry.addData("Blue ", Color_sensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();


            Side.setPosition(.2);


            if (Color_sensor.blue() > 300 && Color_sensor.red() < 300) {

                right.setPower(.5);
                left.setPower(-.5);
                sleep(2000);
                right.setPower(0);
                left.setPower(0);
                sleep(30000);

            }
            else if (Color_sensor.red() > 300 && Color_sensor.blue() < 300) {

                right.setPower(-.5);
                left.setPower(.5);
                sleep(2000);
                right.setPower(0);
                left.setPower(0);
                sleep(30000);
            }

        }

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
            }
        }

