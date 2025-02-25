package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Disabled
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {


    private Encoder leftEncoder, rightEncoder, frontEncoder;
    public double lEncoder, rEncdoder,fEncoder;


    double CIRCUM = 1.96 * Math.PI;
    int TICKS = 8192;
    double TURNING_RAD = 8 * Math.PI;


    @Override

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BackRightMotor"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FrontRightMotor"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BackLeftMotor"));



        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        lEncoder = leftEncoder.getCurrentPosition();
        rEncdoder = rightEncoder.getCurrentPosition();
        fEncoder = frontEncoder.getCurrentPosition();



        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();



            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("leftEncoder: ", leftEncoder.getCurrentPosition());

            telemetry.addData("rightEncoder: ", rightEncoder.getCurrentPosition());

            telemetry.addData("frontEncoder: ", frontEncoder.getCurrentPosition());

            telemetry.update();
        }
    }

    public void getCurrentLocation(double front,double left,double right){

        double heading = (((front/TICKS)*CIRCUM)/TURNING_RAD) * 2 * Math.PI;
        double x_coordinate = ((((right + left)/2)/TICKS)*CIRCUM) * Math.cos(heading);
        double y_coordinate = ((((front))/TICKS)*CIRCUM);

    }

}
