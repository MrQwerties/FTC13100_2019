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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic driving", group="Iterative Opmode")
//@Disabled
public class BasicDriving extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    //Declare motors and servoes
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor lift = null;
    private DcMotor lift2 = null;
    private DcMotor box = null;

    private Servo intake = null;

    //Constants
    final private double LIFT_POWER = 1.0;

    //Driving constants
    final private double TURN_EXPONENT = 1.5;

    //Which side is front
    final private int DIRECTION_ARM = 1;
    final private int DIRECTION_HUB = 2;

    private int frontDirection = DIRECTION_ARM;

    //Driving mode (tank/POV)
    final private int DRIVE_MODE_TANK = 1;
    final private int DRIVE_MODE_POV = 2;

    private int driveMode = DRIVE_MODE_POV;

    private final boolean COMPETITION_MODE = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        //elevator = hardwareMap.get(DcMotor.class, "elevator");
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        box = hardwareMap.get(DcMotor.class, "flippy_box");

        intake = hardwareMap.get(Servo.class, "intake");

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

        box.setDirection(DcMotor.Direction.FORWARD);
        box.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        double left = 0;
        double right = 0;
        if(driveMode == DRIVE_MODE_POV){
            double drive = - gamepad1.left_stick_y;
            double turn = - gamepad1.left_stick_x;
            double turnAmount = - Math.signum(turn) * Math.pow(Math.abs(turn), TURN_EXPONENT);
            left = drive + turnAmount;
            right = drive - turnAmount;
        } else if(driveMode == DRIVE_MODE_TANK){
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;
        } else{ //Just go with POV drive
            double drive = - gamepad1.left_stick_y;
            double turn = - gamepad1.left_stick_x;
            double turnAmount = - Math.signum(turn) * Math.pow(Math.abs(turn), TURN_EXPONENT);
            left = drive + turnAmount;
            right = drive - turnAmount;
        }
        setDrivePowers(left, right);

        if(gamepad1.a){
            box.setPower(1.0);
        } else if(gamepad1.b){
            box.setPower(-1.0);
        } else{
            box.setPower(0);
        }

        if(gamepad1.x){
            lift.setPower(LIFT_POWER);
            lift2.setPower(LIFT_POWER);
        } else if(gamepad1.y){
            lift.setPower(-LIFT_POWER);
            lift2.setPower(-LIFT_POWER);
        } else{
            lift.setPower(0);
            lift2.setPower(0);
        }

        if(gamepad1.dpad_down){
            intake.setPosition(0);
        } else if (gamepad1.dpad_up){
            intake.setPosition(1);
        } else{
            intake.setPosition(0.5);
        }

        if(gamepad1.dpad_left){
            box.setPower(-1);
        } else if (gamepad1.dpad_right){
            box.setPower(1);
        } else{
            box.setPower(0);
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Arm 1", Integer.toString(lift.getCurrentPosition()));
        telemetry.addData("Arm 2", Integer.toString(lift2.getCurrentPosition()));

        if(frontDirection == DIRECTION_ARM){

        }
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

}
