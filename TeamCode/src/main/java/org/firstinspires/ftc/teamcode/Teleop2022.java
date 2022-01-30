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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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


@TeleOp(name = "Teleop2022", group = "Concept")


public class Teleop2022 extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontleftmotor = null;
    private DcMotor frontrightmotor = null;
    private DcMotor backleftmotor = null;
    private DcMotor backrightmotor = null;

    private DcMotor teamMarkerMotor = null;
    private DcMotor output = null;
    private DcMotor input = null;

    private Servo output2 = null;
    private CRServo carouselArm = null;
    private Servo teamMarkerServo = null;


    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Working");
        telemetry.update();

        Thread  driveThread = new DriveThread();

        frontleftmotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        frontrightmotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        backrightmotor = hardwareMap.get(DcMotor.class, "BackRightMotor");
        backleftmotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");

        teamMarkerMotor = hardwareMap.get(DcMotor.class, "TeamMarkerMotor");
        output = hardwareMap.get(DcMotor.class, "Output");
        input = hardwareMap.get(DcMotor.class, "InputMotor");
        output2 = hardwareMap.get(Servo.class, "OutputServo");
        carouselArm = hardwareMap.get(CRServo.class, "CarouselArmServo");
        teamMarkerServo = hardwareMap.get(Servo.class, "TeamMarkerServo");


        frontleftmotor.setPower(0.0);
        frontrightmotor.setPower(0.0);
        backleftmotor.setPower(0.0);
        backrightmotor.setPower(0.0);


        // transfer.setPosition((-0.5));
        // RESET THESE VALUES
        carouselArm.setPower(0.0);
        //teamMarkerServo.setPosition(0.8);



        frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        output2.setPosition(0.7);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        driveThread.start();

        runtime.reset();
        double constant1 = 0.6;
        int changeMove = 0;
        boolean beforeAPressed = false;
        boolean inputRunning = false;
        boolean beforeBPressed = false;
        boolean beforeXPressed = false;
        boolean inputOut = false;
        boolean outputRunning = false;
        boolean beforeYPressed = false;
        int outputState = 0;
        int teamMarkerServoState = 0;
        int outputSlidesState = 0;

        output2.setPosition(0.7);
        teamMarkerServo.setPosition(0.0);





        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //This moves the base
//            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
//            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
//            double rightX = gamepad1.right_stick_x;
//            final double v1 = -r * Math.sin(robotAngle) + rightX;
//            final double v2 = r * Math.cos(robotAngle) + rightX;
//            final double v3 = -r * Math.cos(robotAngle) + rightX;
//            final double v4 = r * Math.sin(robotAngle) + rightX;
//
//            frontleftmotor.setPower(v1);
//            frontrightmotor.setPower(v2);
//            backleftmotor.setPower(v3);
//            backrightmotor.setPower(v4);

            //This moves the base
            /**
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = -gamepad1.left_stick_y + rightX;
            final double v2 = gamepad1.left_stick_x + rightX;
            final double v3 = -gamepad1.left_stick_x + rightX;
            final double v4 = gamepad1.left_stick_y + rightX;
**/



            if (beforeAPressed && beforeAPressed != gamepad1.a) {
                if(inputRunning) {
                    input.setPower(-0.8);
                } else {
                    input.setPower(0.0);
                }
                inputRunning = !inputRunning;
            }
            beforeAPressed = gamepad1.a;


            if (beforeBPressed && beforeBPressed != gamepad1.b) {
                if(outputRunning) {
                    carouselArm.setPower(-1.0);
                } else {
                    carouselArm.setPower(0.0);
                }
                outputRunning = !outputRunning;
            }
            beforeBPressed = gamepad1.b;

            if (beforeXPressed && beforeXPressed != gamepad1.x) {
                if(inputOut) {
                    input.setPower(0.8);
                } else {
                    input.setPower(0.0);
                }
                inputOut = !inputOut;
            }
            beforeXPressed = gamepad1.x;



            if(gamepad2.dpad_up){
                teamMarkerMotor.setPower(-0.3);
            }

            else if(gamepad2.dpad_down){

                teamMarkerMotor.setPower(0.3);

            }
            else{
                teamMarkerMotor.setPower(0.0);
            }

            if(outputState == 0 && gamepad2.b){
                output2.setPosition(0.7);
                outputState = 1;
                sleep(500);
            }
            else if(outputState == 1 && gamepad2.b){
                output2.setPosition(0.3);
                outputState = 0;
                sleep(500);
            }

            if(teamMarkerServoState == 0 && gamepad2.x){
                teamMarkerServo.setPosition(0.5);
                teamMarkerServoState = 1;
                sleep(500);
            }
            else if(teamMarkerServoState == 1 && gamepad2.x){
                teamMarkerServo.setPosition(0.1);
                teamMarkerServoState = 0;
                sleep(500);
            }

            if(outputSlidesState == 0 && gamepad2.left_bumper){
                outputSlidesState = 1;
                output2.setPosition(0.3);
                output.setPower(0.5);
                sleep(1200);
                output.setPower(0.0);
            }
            else if(outputSlidesState == 1 && gamepad2.right_bumper){
                outputSlidesState = 0;
                output2.setPosition(0.4);
                output.setPower(-0.5);
                sleep(1500);
                output.setPower(0.0);
            }
            else{
                output.setPower(0.0);
            }










        idle();

        }
        driveThread.interrupt();
    }
    private class DriveThread extends Thread
    {
        public DriveThread()
        {
            this.setName("DriveThread");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {

            try
            {
                while (!isInterrupted())
                {
                    // we record the Y values in the main class to make showing them in telemetry
                    // easier.

                    double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                    double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                    double rightX = gamepad1.right_stick_x;
                    final double v1 = -r * Math.sin(robotAngle) + rightX;
                    final double v2 = r * Math.cos(robotAngle) + rightX;
                    final double v3 = -r * Math.cos(robotAngle) + rightX;
                    final double v4 = r * Math.sin(robotAngle) + rightX;

                    frontleftmotor.setPower(v1);
                    frontrightmotor.setPower(v2);
                    backleftmotor.setPower(v3);
                    backrightmotor.setPower(v4);

                    idle();
                }
            }
            catch (Exception e){    

            }

        }
    }


}

