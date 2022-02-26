package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Path", group = "Concept")
public class Path1 extends LinearOpMode {
    SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

    @Override

    public void runOpMode(){

        waitForStart();

        Pose2d startPose = new Pose2d(0,0, Math.toRadians(90));
        drivetrain.setPoseEstimate(startPose);
        Trajectory traj2 = drivetrain.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-36, -25, Math.toRadians(270)))
                .build();
        drivetrain.followTrajectory(traj2);



    }
    public void parkingPath() {
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(90));
        drivetrain.setPoseEstimate(startPose);
        sleep(1000);
        // senseing da things

        Trajectory traj1 = drivetrain.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-41.5, 4))
                .build();
        drivetrain.followTrajectory(traj1);

        // moves to the shipping thing


        Trajectory traj2 = drivetrain.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-36, -25, Math.toRadians(270)))
                .build();
        drivetrain.followTrajectory(traj2);
        // duck go brrr


        sleep(1000);

        Trajectory traj3 = drivetrain.trajectoryBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(5.5, 69))
                .build();
        drivetrain.followTrajectory(traj3);
// time to park

        Trajectory traj4 = drivetrain.trajectoryBuilder(traj3.end())
                .forward(31)
                .build();
        drivetrain.followTrajectory(traj4);
// time to park
    }

    public void fullPath(){
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(90));
        drivetrain.setPoseEstimate(startPose);

        sleep(1000);
        // senseing da things

        Trajectory traj1 = drivetrain.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-41.5, 4))
                .build();
        drivetrain.followTrajectory(traj1);

        // moves to the shipping thing


        Trajectory traj2 = drivetrain.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-36, -25,Math.toRadians(270)))
                .build();
        drivetrain.followTrajectory(traj2);
        // duck go brrr


        sleep(1000);

        Trajectory traj3 = drivetrain.trajectoryBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(5.5, 69))
                .build();
        drivetrain.followTrajectory(traj3);
// cycling

        Trajectory traj4 = drivetrain.trajectoryBuilder(traj3.end())
                .forward(31)
                .build();
        drivetrain.followTrajectory(traj4);


        Trajectory traj5 = drivetrain.trajectoryBuilder(traj4.end())
                .back(35)
                .build();
        drivetrain.followTrajectory(traj5);

        Trajectory traj6 = drivetrain.trajectoryBuilder(traj5.end())
                .strafeLeft(41.5)
                .build();
        drivetrain.followTrajectory(traj5);

// time to park
        Trajectory traj7 = drivetrain.trajectoryBuilder(traj6.end())
                .strafeRight(41.5)
                .build();
        drivetrain.followTrajectory(traj7);

        Trajectory traj8 = drivetrain.trajectoryBuilder(traj7.end())
                .forward(35)
                .build();
        drivetrain.followTrajectory(traj8);

    }
}



