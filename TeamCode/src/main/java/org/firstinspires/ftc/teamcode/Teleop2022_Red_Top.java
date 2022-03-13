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


@TeleOp(name = "Teleop2022_Red_Top", group = "Concept")


public class Teleop2022_Red_Top extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    private DcMotor teamMarkerMotor = null;
    private DcMotor output = null;
    private DcMotor input = null;

    private Servo output2 = null;
    private CRServo carouselArm = null;
    private Servo teamMarkerServo = null;

    private DistanceSensor dsensor = null;

    public boolean cycleState = false;
    public double armRestingPosition = 0.4;
    public boolean outOfWearhouse = false;
    public int armState = 0;






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
        teamMarkerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        output.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        teamMarkerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        output2.setPosition(0.3);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        armThread.start();

        runtime.reset();
        boolean beforeAPressed = false;
        boolean inputRunning = false;
        boolean beforeBPressed = false;
        boolean beforeXPressed = false;
        boolean inputOut = false;
        boolean outputRunning = false;


        double currentX = 0.0;
        double currentY = 0.0;
        double currentHeading = 0.0;

        teamMarkerServo.setPosition(0.0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            drivetrain.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y*0.6,
                            -gamepad1.left_stick_x*0.6,
                            -gamepad1.right_stick_x*0.6
                    )
            );

            drivetrain.update();

            Pose2d poseEstimate = drivetrain.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            Pose2d startPose = new Pose2d(10.0 , 0.0, Math.toRadians(0));

            // drivetrain.setPoseEstimate(startPose);
            currentX = poseEstimate.getX();
            currentY = poseEstimate.getY();
            currentHeading = poseEstimate.getHeading();
            // building the trajectories
            Trajectory Traj1 = drivetrain.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(-20,0), SampleMecanumDrive.getVelocityConstraint(55, 238.72114843277868, 11.326),
                            SampleMecanumDrive.getAccelerationConstraint(50))
                    .build();
            Trajectory Traj2 = drivetrain.trajectoryBuilder(Traj1.end())
                    .lineToLinearHeading(new Pose2d(-33.0 , 21.5, Math.toRadians(-80)))
                    .build();

            Trajectory Traj3 = drivetrain.trajectoryBuilder(Traj2.end())
                    .lineToLinearHeading(new Pose2d(-20.0 , -3.0, Math.toRadians(0)))
                    .build();

            Trajectory Traj4 = drivetrain.trajectoryBuilder(Traj3.end())
                    .lineToLinearHeading(new Pose2d(20,-3), SampleMecanumDrive.getVelocityConstraint(55, 238.72114843277868, 11.326),
                            SampleMecanumDrive.getAccelerationConstraint(50))
                    .build();

            Trajectory Traj5 = drivetrain.trajectoryBuilder(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeading()))
                    .lineToLinearHeading(new Pose2d( 1.0, 1.0, 1.0))
                    .build();


            if(gamepad1.dpad_up && !outOfWearhouse){
                outOfWearhouse = true;
                armState = 2;
                drivetrain.followTrajectory(Traj1);
                drivetrain.followTrajectory(Traj2);

            }
            if(gamepad2.y && outOfWearhouse){
                outOfWearhouse = false;
                armState = 1;
                sleep(500);
                drivetrain.followTrajectory(Traj3);
                drivetrain.followTrajectory(Traj4);

                currentX = poseEstimate.getX();
                currentY = poseEstimate.getY();
                currentHeading = poseEstimate.getHeading();
                cycleState = false;
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

                boolean armUp = false;

                while (!isInterrupted())
                {
                    // we record the Y values in the main class to make showing them in telemetry
                    // easier.
                    if(armState == 1){
                        armState = 0;
                        armUp = false;
                        input.setPower(-0.8);
                        output.setTargetPosition(0);
                        output2.setPosition(0.3);
                        armRestingPosition = 0.4;
                        output.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        output.setPower(0.7);
                        while (opModeIsActive() && (output.isBusy())) {

                        }
                        output.setPower(0.0);
                        output.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    if(armState == 2){
                        armState = 0;
                        output2.setPosition(0.5);
                        armRestingPosition = 0.5;
                        output.setTargetPosition(-2300);
                        output.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        output.setPower(-0.7);
                        while (opModeIsActive() && (output.isBusy())) {

                        }
                        output.setPower(0.0);
                        output.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        input.setPower(0.0);
                    }

                    if(dsensor.getDistance(DistanceUnit.CM) < 7.0){
                        output2.setPosition(armRestingPosition);
                        input.setPower(0.5);
                        if(!cycleState){
                            cycleState = true;
                            carouselArm.setPower(0.5);
                            sleep(500);
                            carouselArm.setPower(0.0);
                        }
                    }
                    else{
                        output2.setPosition(0.3);

                    }

                    if(gamepad2.dpad_up){
                        teamMarkerMotor.setPower(0.4);
                        while (opModeIsActive() && (teamMarkerMotor.isBusy())) {

                        }
                        teamMarkerMotor.setPower(0.0);
                    }

                    else if(gamepad2.dpad_down){
                        teamMarkerMotor.setPower(-0.3);
                        while (opModeIsActive() && (teamMarkerMotor.isBusy())) {

                        }
                        teamMarkerMotor.setPower(0.0);

                    }
                    else{
                        teamMarkerMotor.setPower(0.0);
                    }

                    if(gamepad2.y){
                        output2.setPosition(0.7);
                        sleep(1000);
                    }

                    if((teamMarkerServoState == 0) && gamepad2.x){
                        teamMarkerServo.setPosition(0.5);
                        teamMarkerServoState = 1;
                        sleep(500);
                    }
                    else if((teamMarkerServoState == 1) && gamepad2.x){
                        teamMarkerServo.setPosition(0.3);
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

