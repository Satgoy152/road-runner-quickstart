/*
Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *23
 * Redistributions in binary form must reproduce  /`the above copyright notice, this
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
 **/

@Disabled
@TeleOp(name = "AllDirection", group = "Concept")


public class AllDirection extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontleftmotor = null;
    private DcMotor frontrightmotor = null;
    private DcMotor backleftmotor = null;
    private DcMotor backrightmotor = null;
    private DcMotor input = null;
    private DcMotor output = null;
    private DcMotor wobbleArm = null;
    private DcMotor input2 = null;
    private Servo topClaw = null;
    private Servo bottomClaw = null;
    private Servo transfer = null;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Working");
        telemetry.update();



        frontleftmotor = hardwareMap.get(DcMotor.class, "frontleftmotor");
        frontrightmotor = hardwareMap.get(DcMotor.class, "frontrightmotor");
        backrightmotor = hardwareMap.get(DcMotor.class, "backrightmotor");
        backleftmotor = hardwareMap.get(DcMotor.class, "backleftmotor");
        input = hardwareMap.get(DcMotor.class, "input");
        output = hardwareMap.get(DcMotor.class, "output");
        wobbleArm = hardwareMap.get(DcMotor.class, "wobbleArm");
        transfer = hardwareMap.get(Servo.class, "transferServo");
        topClaw = hardwareMap.get(Servo.class, "topClaw");
        bottomClaw = hardwareMap.get(Servo.class, "bottomClaw");
        input2 = hardwareMap.get(DcMotor.class, "input2");





        frontleftmotor.setPower(0.0);
        frontrightmotor.setPower(0.0);
        backleftmotor.setPower(0.0);
        backrightmotor.setPower(0.0);


        // transfer.setPosition((-0.5));
        topClaw.setPosition(0.6);
        bottomClaw.setPosition(0.1);
        transfer.setPosition(0.8);



        frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);






        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double constant1 = 0.6;
        int changeMove = 0;
        boolean beforeAPressed = false;
        boolean inputRunning = false;
        boolean beforeBPressed = false;
        boolean outputRunning = false;
        boolean beforeYPressed = false;
        boolean transferPushing = false;
        boolean beforeRightBPressed = false;
        boolean clawGrabbing = false;
        double reverse = 1.0;
        boolean beforeLeftBPressed = false;
        boolean robotBackwards = false;




        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            

            //This moves the base
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = -r * Math.sin(robotAngle) + rightX;
            final double v2 = r * Math.cos(robotAngle) + rightX;
            final double v3 = -r * Math.cos(robotAngle) + rightX;
            final double v4 = r * Math.sin(robotAngle) + rightX;


            if(gamepad1.x){
                changeMove++;
            }

            if((changeMove % 2) == 0){
                frontleftmotor.setPower(v1*reverse);
                frontrightmotor.setPower(v2*reverse);
                backleftmotor.setPower(v3*reverse);
                backrightmotor.setPower(v4*reverse);
            }
            else{
                frontleftmotor.setPower(v1*constant1*reverse);
                frontrightmotor.setPower(v2*constant1*reverse);
                backleftmotor.setPower(v3*constant1*reverse);
                backrightmotor.setPower(v4*constant1*reverse);
            }


            if (beforeLeftBPressed && beforeLeftBPressed != gamepad1.left_bumper) {
                if(robotBackwards) {
                    reverse = 1.0;
                } else {
                    reverse = -1.0;
                }
                robotBackwards = !robotBackwards;
            }
            beforeLeftBPressed = gamepad1.left_bumper;

            if (beforeAPressed && beforeAPressed != gamepad1.a) {
                if(inputRunning) {
                    input.setPower(0.7);
                    input2.setPower(1.0);
                } else {
                    input.setPower(0.0);
                    input2.setPower(0.0);
                }
                inputRunning = !inputRunning;
            }
            beforeAPressed = gamepad1.a;


            if (beforeBPressed && beforeBPressed != gamepad1.b) {
                if(outputRunning) {
                    output.setPower(1.0);
                } else {
                    output.setPower(0.0);
                }
                outputRunning = !outputRunning;
            }
            beforeBPressed = gamepad1.b;



            if(gamepad1.dpad_up){
                wobbleArm.setPower(1.0);
            }
            else if(gamepad1.dpad_down){
                wobbleArm.setPower(-0.6);
            }
            else{
                wobbleArm.setPower(0.2);
            }

            if (beforeRightBPressed && beforeRightBPressed != gamepad1.right_bumper) {
                if(clawGrabbing) {
                    topClaw.setPosition(0.23);
                    bottomClaw.setPosition(0.4);
                }
                else {
                    topClaw.setPosition(0.6);
                    bottomClaw.setPosition(0.1);
                }
                clawGrabbing = !clawGrabbing;
            }
            beforeRightBPressed = gamepad1.right_bumper;





            if (beforeYPressed && beforeYPressed != gamepad1.y) {
                if(transferPushing){
                    transfer.setPosition(0.7);
                    sleep(50);
                    transfer.setPosition(0.8);

                } else {
                    transfer.setPosition(0.7);
                    sleep(50);
                    transfer.setPosition(0.8);
                }
                transferPushing = !transferPushing;
            }
            beforeYPressed = gamepad1.y;







            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Mode", (changeMove%2));
            //telemetry.addData("Motors", "one (%.2f), two (%.2f), servoone (%.2f), servotwo (%.2f)", onePower, twoPower, servo1pos, servo2pos);
            telemetry.update();
        }
    }
}