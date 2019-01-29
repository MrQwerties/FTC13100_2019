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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Advanced Driving - Version 1", group="Iterative Opmode")
//@Disabled
public class AdvancedDrivingV1 extends OpMode
{
    //Track the runtime
    private ElapsedTime runtime = new ElapsedTime();

    //Declare motors and servos
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor lift = null;
    private DcMotor lift2 = null;
    private DcMotor climber = null;
    private Servo intake = null;
    private Servo lock = null;
    //Accelerometer stuff
    private BNO055IMU imu;
    private Orientation angles;

    //Constants
    final private double LIFT_POWER = 1.0;

    //Driving constants
    final private double TURN_EXPONENT = 1.5;

    //Which side is front
    final private int DIRECTION_ARM = 0;
    final private int DIRECTION_HUB = 1;
    final private String[] DIRECTION_NAMES = {"Arm", "Hub"};

    //Driving mode (tank/POV)
    final private int DRIVE_MODE_TANK = 0;
    final private int DRIVE_MODE_POV = 1;
    final private String[] DRIVE_MODE_NAMES = {"Tank", "POV"};

    //Whether or not it's competition day yet
    private final boolean COMPETITION_MODE = false;

    //Various settings
    private int frontDirection = DIRECTION_ARM; //Can be DIRECTION_ARM or DIRECTION_HUB
    private int driveMode = DRIVE_MODE_POV; //Can be DRIVE_MODE_TANK or DRIVE_MODE_POV
    private boolean override = false;

    //Other runtime variables
    private boolean overrideToggled = false;

    @Override
    public void init() {
        //Get all of the motors and servos
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        climber = hardwareMap.get(DcMotor.class, "climber");
        intake = hardwareMap.get(Servo.class, "intake");
        lock = hardwareMap.get(Servo.class, "lock");

        // Set up drive motors
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        //Encoder stuff
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set up arm motors
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift2.setDirection(DcMotor.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Arm motor encoder stuff
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        climber.setDirection(DcMotorSimple.Direction.FORWARD);
        climber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set up accelerometer
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        //Set driving powers
        double left;
        double right;
        if(driveMode == DRIVE_MODE_POV){
            double drive = - gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double turnAmount = - Math.signum(turn) * Math.pow(Math.abs(turn), TURN_EXPONENT);
            left = drive + turnAmount;
            right = drive - turnAmount;
        } else if(driveMode == DRIVE_MODE_TANK){
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;
        } else{ //Just go with POV drive if something breaks
            double drive = - gamepad1.left_stick_y;
            double turn = - gamepad1.left_stick_x;
            double turnAmount = - Math.signum(turn) * Math.pow(Math.abs(turn), TURN_EXPONENT);
            left = drive + turnAmount;
            right = drive - turnAmount;
        }
        setDrivePowers(left, right);

        //Control the arm
        if(gamepad1.right_trigger > 0.5){
            lift.setPower(LIFT_POWER);
            lift2.setPower(LIFT_POWER);
        } else if(gamepad1.left_trigger > 0.5){
            lift.setPower(-LIFT_POWER);
            lift2.setPower(-LIFT_POWER);
        } else{
            lift.setPower(0);
            lift2.setPower(0);
        }

        //Todo: make bumpers move to position

        //Control the intake
        if(gamepad2.right_bumper){
            intake.setPosition(0);
        } else if (gamepad2.left_bumper){
            intake.setPosition(1);
        } else{
            intake.setPosition(0.5);
        }

        //Control the lock
        if(gamepad2.y){
            lock.setPosition(0);
        } else if (gamepad2.b){
            lock.setPosition(1);
        } else{
            lock.setPosition(0.5);
        }

        //Toggle override
        if(gamepad1.x && !overrideToggled){
            override = !override;
            overrideToggled = true;
        } else{
            overrideToggled = false;
        }

        if(gamepad2.a){
            climber.setPower(1);
        } else if(gamepad2.x){
            climber.setPower(-1);
        } else{
            climber.setPower(0);
        }

        //Check if tilted, and if so, fix it!
        if(!override && false) { //todo: fix values
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            //TODO: you might have to swap the 1's and -1's
            if (angles.secondAngle > 15) {
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            } else if (angles.secondAngle < -15) {
                leftDrive.setPower(-1);
                rightDrive.setPower(-1);
            }
        }

        //Update the telemetry
        telemetry.addData("Run time", runTimeString(runtime.time()));
        telemetry.addData("Arm 1", Integer.toString(lift.getCurrentPosition()));
        telemetry.addData("Arm 2", Integer.toString(lift2.getCurrentPosition()));

        try {
            telemetry.addData("Drive mode", DRIVE_MODE_NAMES[driveMode]);
        } catch(Exception e){
            telemetry.addData("Drive mode", "???");
        }

        try {
            telemetry.addData("Front side", DIRECTION_NAMES[frontDirection]);
        } catch(Exception e){
            telemetry.addData("Front side", "???");
        }

        telemetry.addData("Manual override", override);

        telemetry.update();
    }

    @Override
    public void stop() {
        if(COMPETITION_MODE){
            telemetry.clearAll();
            telemetry.addData("So the match's over", "How'd it go?");
        }
    }

    public void setDrivePowers(double left, double right){
        left = Range.clip(left, -1.0, 1.0);
        right = Range.clip(right, -1.0, 1.0);
        if(frontDirection == DIRECTION_HUB){
            leftDrive.setPower(left);
            rightDrive.setPower(right);
        } else if(frontDirection == DIRECTION_ARM){
            leftDrive.setPower(-right);
            rightDrive.setPower(-left);
        } else{
            leftDrive.setPower(left);
            rightDrive.setPower(right);
        }
    }

    public static String runTimeString(double time){
        int minutes = (int) time/60;
        double seconds = time - 60 * minutes;
        return Integer.toString(minutes) + " m " + String.format("%.2f", seconds) + " s";
    }

}
