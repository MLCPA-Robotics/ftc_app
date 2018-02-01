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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="Vumark_RED_RIGHT'", group ="Concept")
@Disabled
public class DO_NOT_TOUCH_ME_10405_RED_RIGHT extends LinearOpMode {

    private DcMotor right = null;
    private DcMotor left = null;
    private DcMotor lift = null;

    private DcMotor open_close = null;
    private Servo Side;


    private ColorSensor Color_sensor;
    private DeviceInterfaceModule cdim;
    static final int LED_CHANNEL = 5;


    private ElapsedTime runtime = new ElapsedTime();

    public static final String TAG = "Vuforia VuMark Sample";
    private OpenGLMatrix lastLocation = null;
    private double tX;
    private double tY;
    private double tZ;

    private double rX;
    private double rY;
    private double rZ;
    private VuforiaLocalizer vuforia;


    public void forward_backward(double d){
        left.setPower(d);
        right.setPower(d);
    }



    @Override public void runOpMode() {
        // Color sensor
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        right = hardwareMap.dcMotor.get("Right");
        left = hardwareMap.dcMotor.get("Left");
        lift = hardwareMap.dcMotor.get("Lift");

        open_close = hardwareMap.dcMotor.get("Open_close");
        Side = hardwareMap.servo.get("Side");


        right.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);

        // Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Ad52oDb/////AAAAGdOx1OONeUSzjgVBj4X88JVEJLviRNUjnzZv+D3h6Df+4IbhD1ilHwSlLfXHxaKJrrG+lqQBkcuDRidgEOQ8omkGob+fZg1B8URRrY7vKsvqIIimPrPE3JCFg7RNgbn0ecg2Duf/CLj+4MncIGG28nfTCN3t25ccDJRgd3UkOfuidMeJ8G1O0o0VoLMzAODpiUc4il7gpmaxvDHC6OpWjyg5KACqTC3E8/Sc0iH65r4mkOqP/VpqB7GoKpWnBc12sD4JvsnxzEG2xJF2OD4zVdJuqTJIxVlW9zRQMsX/rUqy45/GdsPMxS0HhDSUmE9m4o3IQ8yNHJdLliP2AKfyRdoA8HtGEfLOrMNOEXZDdnrr";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        runtime.reset();
        waitForStart();


        relicTrackables.activate();

        while (opModeIsActive()) {
            if (runtime.time() >23 ){

                if (Color_sensor.blue() > Color_sensor.red()){
                    telemetry.addData("BLUE DETECTED", Color_sensor.blue());
                    right.setPower(-.5);
                    left.setPower(.5);
                    sleep(10000);
                }
                else if (Color_sensor.red() > Color_sensor.blue()){

                    telemetry.addData("RED DETECTED", Color_sensor.red());
                    right.setPower(.5);
                    left.setPower(-.5);
                    sleep(10000);
                }
            }

            if (runtime.time() < 22) {
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                    telemetry.addData("Pose", format(pose));


                    if (pose != null) {
                        VectorF trans = pose.getTranslation();
                        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                        //Extract the X,Y and Z compnents of the offset of the target relative to the robot
                        tX = trans.get(0);
                        tY = trans.get(1);
                        tZ = trans.get(2);

                        rX = rot.firstAngle;
                        rY = rot.secondAngle;
                        rZ = rot.thirdAngle;

                    }
                    if (vuMark == RelicRecoveryVuMark.LEFT) {
                        telemetry.addData("Vumark is identified:", "LEFT");
                        telemetry.addData("X =", tX);
                        telemetry.addData("Y =", tY);
                        telemetry.addData("Z =", tZ);

                        forward_backward(.5);
                        sleep(500);
                        left.setPower(.5);
                        sleep(800);
                        forward_backward(.6);
                        sleep(2000);


                    } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                        telemetry.addData("Vumark is identified:", "CENTER");
                        telemetry.addData("X =", tX);
                        telemetry.addData("Y =", tY);
                        telemetry.addData("Z =", tZ);
                        forward_backward(.3);
                        sleep(2000);

                    } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                        telemetry.addData("Vumark is identified:", "RIGHT");
                        telemetry.addData("X =", tX);
                        telemetry.addData("Y =", tY);
                        telemetry.addData("Z =", tZ);

                        forward_backward(.5);
                        sleep(1000);
                    }
                } else {
                    telemetry.addData("VuMark", "not visible");
                }

                telemetry.update();
            }
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
