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

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


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

@Autonomous(name="Auto test: Landing only", group="Linear Opmode")
//@Disabled
public class AutoLandingOnly extends LinearOpMode {
    //TensorFlow stuff
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final int MINERAL_NONE = 0;
    private static final int MINERAL_GOLD = 1;
    private static final int MINERAL_SILVER = 2;

    private static final String[] MINERAL_CODES = {"NONE", "GOLD", "SILVER"};

    private static final String VUFORIA_KEY = "ATn1lRr/////AAAAmduMNcrdlkCLulidDd6hjBN+FyfK3LlDqPIN1RT6wMezngQ22dsI18ZVSWRjE4xszgNdh3OT6Ypso+oxSa+EMMFO+P4oO2CFWnrn5eWf8cL0kX7dhoIsbbWbvU7YcdESclpSxglh8ft6S5AScjCrwC/75go8HBw+WS8W2zQbt72cSb3SmHESSkykpMM1+Wi3uHw2bTqUoQznvIDBQUdyebvZ6pWhdcY97dZRBAFELkukLX3OihLUdezI5q80kx9M05YLxXfug/62ojkVUhy4YOTEMQQ3c4g4kMBuKNOdK6vzXeejIgYjl+DoXFdci8tbAVpzUHDo631BHh3Yq/IntPRZmseHf5gtxblHZq/fhOtM";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor climber = null;

    private Servo intake = null;

    private boolean booped = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        climber = hardwareMap.get(DcMotor.class, "climber");
        climber.setDirection(DcMotorSimple.Direction.FORWARD);
        climber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(Servo.class, "intake");

        //Vision stuff
        initVuforia();
        initTfod();

        booped = false;

        waitForStart();
        tfod.activate();

        //Extend hook
        climber.setPower(1);
        sleep(540);
        climber.setPower(0);
        delay();

        tfod.shutdown();
    }

    private void setPowers(double left, double right, long amount){
        leftDrive.setPower(-left);
        rightDrive.setPower(-right);
        sleep(amount);
    }

    private void delay(long millis){
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(millis);
    }

    private void delay(){
        delay(200);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private int getMineral() {
        delay(2500);
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        delay(2500);
        if(updatedRecognitions != null){
            if(updatedRecognitions.size() > 0){
                Recognition recognition = updatedRecognitions.get(0);
                if(recognition.getLabel().equals(LABEL_GOLD_MINERAL)){
                    return MINERAL_GOLD;
                } else{
                    return MINERAL_SILVER;
                }
            }
        }
        return MINERAL_NONE;
    }

    private void boop(long amount) {
        setPowers(1, 1, amount);
        delay(500);
        setPowers(-0.9, -0.9, amount);
        delay(500);
    }
}
