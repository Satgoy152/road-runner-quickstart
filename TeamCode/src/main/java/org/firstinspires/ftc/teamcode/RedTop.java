package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(name = "Redtop", group = "Concept")
public class RedTop extends LinearOpMode {
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
    private DcMotor teamMarkerMotor = null;
    private DcMotor output = null;
    private DcMotor input = null;

    private Servo output2 = null;
    private CRServo carouselArm = null;
    private Servo teamMarkerServo = null;
    private DistanceSensor dsensor = null;

    private DcMotor FrontRightMotor, BackRightMotor, FrontLeftMotor, BackLeftMotor;

    public boolean cycleState = false;
    public double armRestingPosition = 0.4;
    public boolean outOfWearhouse = false;
    public int armState = 0;

    boolean isDuckDetected = false;
    public int autoLevel = 0;



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

        autoLevel = checkForDuck();

        telemetry.addData("Level: ", autoLevel);

        telemetry.update();



        path(autoLevel);



    }
    public void path(int level) {

        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        Thread armThread = new ArmThread();

        teamMarkerMotor = hardwareMap.get(DcMotor.class, "TeamMarkerMotor");
        output = hardwareMap.get(DcMotor.class, "Output");
        input = hardwareMap.get(DcMotor.class, "InputMotor");
        output2 = hardwareMap.get(Servo.class, "OutputServo");
        carouselArm = hardwareMap.get(CRServo.class, "CarouselArmServo");
        teamMarkerServo = hardwareMap.get(Servo.class, "TeamMarkerServo");
        dsensor = hardwareMap.get(DistanceSensor.class, "dsensor");
        int counter = 0;

        armThread.start();

// -------------------------------------------------- starting pathways ----------------------------------------------------------------------------------
        // creating the pose
        Pose2d startPose = new Pose2d(-20 , 0, Math.toRadians(90));
        // building the trajectories
        Trajectory Traj1 = drivetrain.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-34.0 , 28, Math.toRadians(-80)))
                .build();
        Trajectory Traj2 = drivetrain.trajectoryBuilder(Traj1.end())
                .splineTo(new Vector2d(10 , -16), Math.toRadians(-30))
                .build();
        Trajectory Traj3 = drivetrain.trajectoryBuilder(Traj2.end())
                .lineToLinearHeading(new Pose2d(25.0, -10, Math.toRadians(-60)))
                .build();
        Trajectory Traj4 = drivetrain.trajectoryBuilder(Traj3.end())
                .lineToLinearHeading(new Pose2d(30.0 , -20, Math.toRadians(-60)))
                .build();
        Trajectory Traj5 = drivetrain.trajectoryBuilder(Traj4.end())
                .lineToLinearHeading(new Pose2d(10 , -16, Math.toRadians(-30)))
                .build();
//        Trajectory Traj6 = drivetrain.trajectoryBuilder(Traj5.end())
//                .lineToLinearHeading(new Pose2d(-15,-16,Math.toRadians(-30)))
//                .build();
//        Trajectory Traj7 = drivetrain.trajectoryBuilder(Traj6.end())
//                .lineToLinearHeading(new Pose2d(-19.0 , 20, Math.toRadians(-100)))
//                .build();

//        Trajectory goForward = drivetrain.trajectoryBuilder(Traj3.end())
//                .forward(10, SampleMecanumDrive.getVelocityConstraint(30, 238.72114843277868, 11.326),
//                        SampleMecanumDrive.getAccelerationConstraint(50))
//                .build();
//        Trajectory goBackward = drivetrain.trajectoryBuilder(goForward.end())
//                .back(10, SampleMecanumDrive.getVelocityConstraint(30, 238.72114843277868, 11.326),
//                        SampleMecanumDrive.getAccelerationConstraint(50))
//                .build();


        waitForStart();
        // start of auto
        if(level == 1){
            armState = 3;
        }
        if(level == 2){
            armState = 4;
        }
        if(level == 3){
            armState = 5;
        }
        drivetrain.followTrajectory(Traj1);
        // raise output, turn servo
        armRestingPosition = 0.7;
        sleep(1000);

        for (int i = 0; i < 3; i++) {

            drivetrain.followTrajectory(Traj2);
            armState = 1;
            drivetrain.followTrajectory(Traj3);
            drivetrain.followTrajectory(Traj4);
            drivetrain.followTrajectory(Traj5);
            //drivetrain.followTrajectory(Traj6);
            //drivetrain.followTrajectory(Traj7);
            armState = 2;

            armRestingPosition = 0.7;
            sleep(2000);
            armState = 1;
        }

        armThread.interrupt();

    }// path rung level
//    public void second() {
//        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
//        teamMarkerMotor = hardwareMap.get(DcMotor.class, "TeamMarkerMotor");
//        output = hardwareMap.get(DcMotor.class, "Output");
//        input = hardwareMap.get(DcMotor.class, "InputMotor");
//        output2 = hardwareMap.get(Servo.class, "OutputServo");
//        carouselArm = hardwareMap.get(CRServo.class, "CarouselArmServo");
//        teamMarkerServo = hardwareMap.get(Servo.class, "TeamMarkerServo");
//        dsensor = hardwareMap.get(DistanceSensor.class, "dsensor");
//        int counter = 0;
//
//// -------------------------------------------------- starting pathways ----------------------------------------------------------------------------------
//        // creating the pose
//        Pose2d startPose = new Pose2d(-20 , 0, Math.toRadians(90));
//        // building the trajectories
//        Trajectory Traj1 = drivetrain.trajectoryBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-34.0 , 22.5, Math.toRadians(-70)))
//                .build();
////        Trajectory Traj2 = drivetrain.trajectoryBuilder(Traj1.end())
////                .lineToLinearHeading(new Pose2d(-20.0 , -5.0, Math.toRadians(0)))
////                .build();
////        Trajectory Traj3 = drivetrain.trajectoryBuilder(Traj2.end())
////                .lineToLinearHeading(new Pose2d(8.0 , -5.0, Math.toRadians(0)))
////                .build();
//        Trajectory Traj2 = drivetrain.trajectoryBuilder(Traj1.end())
//                .splineToLinearHeading(new Pose2d(8.0,-5,Math.toRadians(-70)),Math.toRadians(0))
//                .build();
//        Trajectory Traj3 = drivetrain.trajectoryBuilder(Traj2.end())
//                .lineToLinearHeading(new Pose2d(16.0, 3.0, Math.toRadians(-30)))
//                .build();
//        Trajectory Traj4 = drivetrain.trajectoryBuilder(Traj3.end())
//                .lineToLinearHeading(new Pose2d(8.0 , -3.0, Math.toRadians(0)))
//                .build();
//        Trajectory Traj5 = drivetrain.trajectoryBuilder(Traj4.end())
//                .lineToLinearHeading(new Pose2d(-20.0 , -5.0, Math.toRadians(0)))
//                .build();
//        Trajectory Traj6 = drivetrain.trajectoryBuilder(Traj5.end())
//                .lineToLinearHeading(new Pose2d(-34.0 , 22.5, Math.toRadians(-70)))
//                .build();
//        Trajectory goForward = drivetrain.trajectoryBuilder(Traj3.end())
//                .forward(6, SampleMecanumDrive.getVelocityConstraint(15, 238.72114843277868, 11.326),
//                        SampleMecanumDrive.getAccelerationConstraint(50))
//                .build();
//        Trajectory goBackward = drivetrain.trajectoryBuilder(goForward.end())
//                .back(6, SampleMecanumDrive.getVelocityConstraint(15, 238.72114843277868, 11.326),
//                        SampleMecanumDrive.getAccelerationConstraint(50))
//                .build();
//
//
//
//        // start of auto
//        drivetrain.followTrajectory(Traj1);
//        // raise output, turn servo
//        output.setTargetPosition(-1600);
//        output.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        output.setPower(-0.7);
//        output2.setPosition(0.7);
//        sleep(500);
//        output2.setPosition(0.3);
//        output.setTargetPosition(0);
//        output.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        output.setPower(0.7);
//
//        for (int i = 0; i < 3; i++) {
//            drivetrain.followTrajectory(Traj2);
//            input.setPower(-0.9);
//            drivetrain.followTrajectory(Traj3);
//
//            // spin input
//            while (dsensor.getDistance(DistanceUnit.CM) > 7.0) {
//                input.setPower(-0.8);
//                drivetrain.followTrajectory(goForward);
//                counter++;
//                sleep(500);
//            }
//            for (i = 0; i < counter; i++)
//            { drivetrain.followTrajectory(goBackward);}
//            input.setPower(0);
//            output2.setPosition(0.4);
//            drivetrain.followTrajectory(Traj4);
//            drivetrain.followTrajectory(Traj5);
//            drivetrain.followTrajectory(Traj6);
//            output.setTargetPosition(-2300);
//            output.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            output.setPower(-0.7);
//            output2.setPosition(0.7);
//            sleep(500);
//            output2.setPosition(0.3);
//            output.setTargetPosition(0);
//            output.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            output.setPower(0.7);
//
//
//        }
//
//
//
//
//
//    }// second rung level
//    public void third() {
//        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
//        teamMarkerMotor = hardwareMap.get(DcMotor.class, "TeamMarkerMotor");
//        output = hardwareMap.get(DcMotor.class, "Output");
//        input = hardwareMap.get(DcMotor.class, "InputMotor");
//        output2 = hardwareMap.get(Servo.class, "OutputServo");
//        carouselArm = hardwareMap.get(CRServo.class, "CarouselArmServo");
//        teamMarkerServo = hardwareMap.get(Servo.class, "TeamMarkerServo");
//        dsensor = hardwareMap.get(DistanceSensor.class, "dsensor");
//        int counter = 0;
//
//// -------------------------------------------------- starting pathways ----------------------------------------------------------------------------------
//        // creating the pose
//        Pose2d startPose = new Pose2d(-20 , 0, Math.toRadians(90));
//        // building the trajectories
//        Trajectory Traj1 = drivetrain.trajectoryBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-34.0 , 23.5, Math.toRadians(-60)))
//                .build();
//        Trajectory Traj2 = drivetrain.trajectoryBuilder(Traj1.end())
//                .lineToLinearHeading(new Pose2d(-20.0 , -5.0, Math.toRadians(0)))
//                .build();
//        Trajectory Traj3 = drivetrain.trajectoryBuilder(Traj2.end())
//                .lineToLinearHeading(new Pose2d(10.0 , -5.0, Math.toRadians(0)))
//                .build();
//        Trajectory Traj4 = drivetrain.trajectoryBuilder(Traj3.end())
//                .lineToLinearHeading(new Pose2d(16.0, 3.0, Math.toRadians(-30)))
//                .build();
//        Trajectory Traj5 = drivetrain.trajectoryBuilder(Traj4.end())
//                .lineToLinearHeading(new Pose2d(8.0 , -3.0, Math.toRadians(0)))
//                .build();
//        Trajectory Traj6 = drivetrain.trajectoryBuilder(Traj5.end())
//                .lineToLinearHeading(new Pose2d(-20.0 , -5.0, Math.toRadians(0)))
//                .build();
//        Trajectory Traj7 = drivetrain.trajectoryBuilder(Traj6.end())
//                .lineToLinearHeading(new Pose2d(-34.0 , 22.5, Math.toRadians(-70)))
//                .build();
//        Trajectory goForward = drivetrain.trajectoryBuilder(Traj3.end())
//                .forward(6, SampleMecanumDrive.getVelocityConstraint(15, 238.72114843277868, 11.326),
//                        SampleMecanumDrive.getAccelerationConstraint(50))
//                .build();
//        Trajectory goBackward = drivetrain.trajectoryBuilder(goForward.end())
//                .back(6, SampleMecanumDrive.getVelocityConstraint(15, 238.72114843277868, 11.326),
//                        SampleMecanumDrive.getAccelerationConstraint(50))
//                .build();
//
//
//
//        // start of auto
//        drivetrain.followTrajectory(Traj1);
//        // raise output, turn servo
//        output.setTargetPosition(-2300);
//        output.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        output.setPower(-0.7);
//        output2.setPosition(0.7);
//        sleep(500);
//        output2.setPosition(0.3);
//        output.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        output.setPower(0.7);
//
//        for (int i = 0; i < 3; i++) {
//            drivetrain.followTrajectory(Traj2);
//            input.setPower(-0.9);
//            drivetrain.followTrajectory(Traj3);
//            drivetrain.followTrajectory(Traj4);
//            // spin input
//            while (dsensor.getDistance(DistanceUnit.CM) > 7.0) {
//                input.setPower(-0.9);
//                drivetrain.followTrajectory(goForward);
//                counter++;
//                sleep(500);
//            }
//            for (i = 0; i < counter; i++)
//            { drivetrain.followTrajectory(goBackward);}
//            input.setPower(0);
//            output2.setPosition(0.4);
//            drivetrain.followTrajectory(Traj5);
//            drivetrain.followTrajectory(Traj6);
//            drivetrain.followTrajectory(Traj7);
//            output.setTargetPosition(-2300);
//            output.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            output.setPower(-0.7);
//            output2.setPosition(0.7);
//            sleep(500);
//            output2.setPosition(0.3);
//            output.setTargetPosition(0);
//            output.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            output.setPower(0.7);
//
//
//        }
//
//
//
//
//
//    }// third rung level

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

    public int checkForDuck(){
            while (!isDuckDetected) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {

                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
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

                                isDuckDetected = true;
                                telemetry.addData("Object Detected", "Duck");
                                // 191 - 300
                                if(recognition.getRight() <= 550.0 && recognition.getTop() > 200){
                                    // Lift arm to path rung level
                                    telemetry.addData("First Rung Level", ".");
                                    //path();
                                    return 1;
                                    // make this change based on positioning of duck
                                }
                                // 400 - 600
                                else if(recognition.getRight() > 550.0 && recognition.getTop() > 200){
                                    // Lift arm to second rung level
                                    telemetry.addData("Second Rung Level", ".");
                                    //second();
                                    return 2;
                                }
                            }else {
                                //third();
                                telemetry.addData("Third Rung Level", ".");
                                isDuckDetected = false;
                                return 3;
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        return 0;
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

                    if(armState == 3){
                        armState = 0;
                        output2.setPosition(0.4);
                        armRestingPosition = 0.5;
//                        output.setTargetPosition(-300);
//                        output.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        output.setPower(-0.7);
                        while (opModeIsActive() && (output.isBusy())) {

                        }
                        output.setPower(0.0);
                        output.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        input.setPower(0.0);
                        output2.setPosition(0.7);
                    }

                    if(armState == 4){
                        armState = 0;
                        output2.setPosition(0.5);
                        armRestingPosition = 0.5;
                        output.setTargetPosition(-1200);
                        output.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        output.setPower(-0.7);
                        while (opModeIsActive() && (output.isBusy())) {

                        }
                        output.setPower(0.0);
                        output.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        input.setPower(0.0);
                    }

                    if(armState == 5){
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
                    }
                    else{
                        output2.setPosition(0.3);
                    }



                    idle();
                }
            }
            catch (Exception e){

            }

        }
    }

}
