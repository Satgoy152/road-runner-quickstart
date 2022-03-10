package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Disabled
@Autonomous(name = "Path2", group = "Concept")
public class ultimateGoalAuto extends LinearOpMode {
    @Override
    public void runOpMode(){
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        //R and L sides are in relation to when the person is facing the goals
        //if(were blue alliance left side){
        Pose2d startPose = new Pose2d(-60, 48, 0.17453292519943295);
        drivetrain.setPoseEstimate(startPose);

        Trajectory traj1 = drivetrain.trajectoryBuilder(startPose)
                .forward(30)
                .strafeRight(13)
                .build();

        drivetrain.followTrajectory(traj1);
        //PAUSE TO SENSE
        //use vuforia to figure out # of rings

        //if(path1) {
        //Trajectories for Path1
        Trajectory traj2 = drivetrain.trajectoryBuilder(traj1.end())
                .strafeLeft(25)
                .forward(42)
                .build();

        Trajectory traj3 = drivetrain.trajectoryBuilder(traj2.end())
                .strafeRight(25)
                .back(11)
                .build();

        //PAUSE TO SHOOT
        //shoot rings into goal

        Trajectory traj4 = drivetrain.trajectoryBuilder(traj3.end())
                .forward(11)
                .build();

        //Path1
        drivetrain.followTrajectory(traj2);
        drivetrain.followTrajectory(traj3);
        drivetrain.followTrajectory(traj4);
        //}
            /*else if(path2) {
                //Trajectories for Path2
                Trajectory traj2 = drivetrain.trajectoryBuilder(traj1.end())
                        .forward(65)
                        .build();

                Trajectory traj3 = drivetrain.trajectoryBuilder(traj2.end())
                        .back(35)
                        .build();

                //PAUSE TO SHOOT
                //shoot rings into goal

                Trajectory traj4 = drivetrain.trajectoryBuilder(traj3.end())
                        .forward(11)
                        .build();

                //Path2
                drivetrain.followTrajectory(traj2);
                drivetrain.followTrajectory(traj3);
                drivetrain.followTrajectory(traj4);
            }
            else {
                //Trajectories for Path3
                Trajectory traj2 = drivetrain.trajectoryBuilder(traj1.end())
                        .strafeLeft(25)
                        .forward(90)
                        .build();

                Trajectory traj3 = drivetrain.trajectoryBuilder(traj2.end())
                        .strafeRight(25)
                        .back(60)
                        .build();

                //PAUSE TO SHOOT
                //shoot rings into goal

                Trajectory traj4 = drivetrain.trajectoryBuilder(traj3.end())
                        .forward(11)
                        .build();

                //Path3
                drivetrain.followTrajectory(traj2);
                drivetrain.followTrajectory(traj3);
                drivetrain.followTrajectory(traj4);
            }*/
        //}
    }
}