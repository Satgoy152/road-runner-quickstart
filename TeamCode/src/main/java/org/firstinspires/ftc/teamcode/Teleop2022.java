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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

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

    private Encoder leftEncoder, rightEncoder, frontEncoder;


//    private DcMotor frontleftmotor = null;
//    private DcMotor frontrightmotor = null;
//    private DcMotor backleftmotor = null;
//    private DcMotor backrightmotor = null;

    private DcMotor teamMarkerMotor = null;
    private DcMotor output = null;
    private DcMotor input = null;

    private Servo output2 = null;
    private CRServo carouselArm = null;
    private Servo teamMarkerServo = null;

    private DistanceSensor dsensor = null;



    @Override

    public void runOpMode() {

        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        telemetry.addData("Status", "Working");
        telemetry.update();

        Thread armThread = new ArmThread();


        teamMarkerMotor = hardwareMap.get(DcMotor.class, "TeamMarkerMotor");
        output = hardwareMap.get(DcMotor.class, "Output");

        input = hardwareMap.get(DcMotor.class, "InputMotor");
        output2 = hardwareMap.get(Servo.class, "OutputServo");
        carouselArm = hardwareMap.get(CRServo.class, "CarouselArmServo");
        teamMarkerServo = hardwareMap.get(Servo.class, "TeamMarkerServo");
        dsensor = hardwareMap.get(DistanceSensor.class, "dsensor");


        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BackRightMotor"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FrontRightMotor"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BackLeftMotor"));

        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // RESET THESE VALUES
        carouselArm.setPower(0.0);

        output.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        output.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        output2.setPosition(0.3);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        armThread.start();

        runtime.reset();
        double constant1 = 0.6;
        int changeMove = 0;
        double posUp = 0.7;
        double posDown = 0.3;
        boolean beforeAPressed = false;
        boolean inputRunning = false;
        boolean beforeBPressed = false;
        boolean beforeXPressed = false;
        boolean inputOut = false;
        boolean outputRunning = false;
        boolean beforeYPressed = false;
        int outputState = 0;
        int outputSlidesState = 1;

        teamMarkerServo.setPosition(0.0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            drivetrain.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drivetrain.update();

            Pose2d poseEstimate = drivetrain.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            Pose2d startPose = new Pose2d(0.0 , 0.0, Math.toRadians(0));
            //drivetrain.setPoseEstimate(startPose);
            // building the trajectories
            Trajectory Traj1 = drivetrain.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(-20.0 , 0.0, Math.toRadians(0)))
                    .build();
            Trajectory Traj2 = drivetrain.trajectoryBuilder(Traj1.end())
                    .lineToLinearHeading(new Pose2d(-34.0 , 25.5, Math.toRadians(-70)))
                    .build();

            Trajectory Traj3 = drivetrain.trajectoryBuilder(Traj2.end())
                    .lineToLinearHeading(new Pose2d(-20.0 , -3.0, Math.toRadians(0)))
                    .build();

            Trajectory Traj4 = drivetrain.trajectoryBuilder(Traj3.end())
                    .lineToLinearHeading(new Pose2d(10.0 , -3.0, Math.toRadians(0)))
                    .build();

            Trajectory Traj5 = drivetrain.trajectoryBuilder(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeading()))
                    .lineToLinearHeading(new Pose2d(10.0 , -3.0, Math.toRadians(0)))
                    .build();



            if(gamepad1.dpad_up){

                drivetrain.followTrajectory(Traj1);
                drivetrain.followTrajectory(Traj2);

            }

            if(gamepad1.dpad_down){

                drivetrain.followTrajectory(Traj3);
                drivetrain.followTrajectory(Traj4);

            }

            if(gamepad1.dpad_right){
                drivetrain.followTrajectory(Traj5);
            }

            if (beforeAPressed && beforeAPressed != gamepad1.y) {
                if(inputRunning) {
                    input.setPower(-0.9);
                } else {
                    input.setPower(0.0);
                }
                inputRunning = !inputRunning;
            }
            beforeAPressed = gamepad1.y;

            if (beforeXPressed && beforeXPressed != gamepad1.x) {
                if(inputOut) {
                    input.setPower(0.8);
                } else {
                    input.setPower(0.0);
                }
                inputOut = !inputOut;
            }
            beforeXPressed = gamepad1.x;


            if (beforeBPressed && beforeBPressed != gamepad1.b) {
                if(outputRunning) {
                    carouselArm.setPower(-1.0);
                } else {
                    carouselArm.setPower(0.0);
                }
                outputRunning = !outputRunning;
            }
            beforeBPressed = gamepad1.b;


            if(dsensor.getDistance(DistanceUnit.CM) < 7.0){
                output2.setPosition(0.5);
                output.setPower(0.0);
//                //carouselArm.setPower(0.5);
//                sleep(500);
//                carouselArm.setPower(0.0);
            }
            else{
                output2.setPosition(posDown);
                carouselArm.setPower(0.0);
            }

        idle();

        }


        armThread.interrupt();
    }



    private class ArmThread extends Thread
    {
        public ArmThread()
        {
            this.setName("ArmThread");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override

        public void run()
        {

            try
            {
                int teamMarkerServoState = 0;

                while (!isInterrupted())
                {
                    // we record the Y values in the main class to make showing them in telemetry
                    // easier.
                    if(gamepad1.dpad_down){
                        output.setTargetPosition(0);
                        output2.setPosition(0.3);
                        output.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        output.setPower(0.7);

                        while (opModeIsActive() && (output.isBusy())) {

                        }
                        output.setPower(0.0);
                        output.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        input.setPower(-0.9);
                    }
                    if(gamepad1.dpad_up){
                        input.setPower(-0.2);
                        output.setTargetPosition(-2300);
                        output.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        output.setPower(-0.7);
                        while (opModeIsActive() && (output.isBusy())) {

                        }
                        output.setPower(0.0);
                        output.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        input.setPower(0.0);
                    }


                    if(gamepad2.dpad_up){
                        teamMarkerMotor.setPower(-0.3);
                    }

                    else if(gamepad2.dpad_down){

                        teamMarkerMotor.setPower(0.3);

                    }
                    else{
                        teamMarkerMotor.setPower(0.0);
                    }

                    if(gamepad2.y){
                        output2.setPosition(0.7);
                        sleep(500);
                    }

                    if((teamMarkerServoState == 0) && gamepad2.x){
                        teamMarkerServo.setPosition(0.5);
                        teamMarkerServoState = 1;
                        sleep(500);
                    }
                    else if((teamMarkerServoState == 1) && gamepad2.x){
                        teamMarkerServo.setPosition(0.1);
                        teamMarkerServoState = 0;
                        sleep(500);
                    }


                    idle();
                }
            }
            catch (Exception e){

            }

        }
    }

}

