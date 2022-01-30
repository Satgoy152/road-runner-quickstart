package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(name = "Pathcopy", group = "Concept")
public class Path1copy extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AQNWIuv/////AAABmfvHQHyt3E7Cg10lU7KS2rkivBYNlGOzCiYZjZNpw+NSO2GF9dc8Hr8NtbWKmOoVq1w/9aiky0/+BQGj/HIHEXaXWRgOTo8anhqJ4AjT0OI0o4iSNTa3vL05WT4Vsjr1yYeBENnZFmgtKGBb6whX8lts53gQK3g51q1W6Nug+ccfYVgEVNNOHrRecnQAfabR13muLa5zNvdI5ZD3Thdc1KaBowlMrOAmM/GjV6Gl3+WK6Tb1mN4Cj6KXANHpx37/Nyjx6goXuUXT45qF5r/ozS6FWTqesDS3KhVQ9MVbIIAWv8pldBDlrUinY0yIK9+z/nfGH9mMkqK/cSkpPR9FGP56xhn11BjzRHXYYOZ9zuzh";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

    @Override

    public void runOpMode(){


        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            //tfod.setZoom(2.5, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        checkForDuck();

        // parkingPath();
        //fullPath();


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

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    private void checkForDuck(){
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        boolean isDuckDetected = false;
                        boolean isCubeDetected = false;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            i++;

                            // check label to see if the camera now sees a Duck
                            if (recognition.getLabel().equals("Duck")) {
                                fullPath();
                                parkingPath();
                                isDuckDetected = true;
                                telemetry.addData("Object Detected", "Duck");

                            } else {
                                isDuckDetected = false;
                            }
                            if (recognition.getLabel().equals("Cube")) {
                                isCubeDetected = true;
                                telemetry.addData("Object Detected", "Cube");
                            } else {
                                isCubeDetected = false;
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }
    }
}



