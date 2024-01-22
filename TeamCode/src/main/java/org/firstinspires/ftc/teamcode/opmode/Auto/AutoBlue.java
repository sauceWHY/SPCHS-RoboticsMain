package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.common.hardware.hardwareinit.armmotor;
import static org.firstinspires.ftc.teamcode.common.hardware.hardwareinit.leftSlide;
import static org.firstinspires.ftc.teamcode.common.hardware.hardwareinit.rightSlide;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.subsystems.RightClawSubsystem;
import org.firstinspires.ftc.teamcode.common.Vision.ContourPipelineBlue;
import org.firstinspires.ftc.teamcode.common.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "AutoBlue", group = "org/firstinspires/ftc/teamcode/drive/opmode/Competition")
public class AutoBlue extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private static ServoImplEx leftClaw;
    private static ServoImplEx rightClaw;
    private static final int CAMERA_WIDTH = 640; // width  of camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of camera resolution
    public static double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 180.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 96, 255);
    private OpenCvCamera webcam;
    public static PIDController controller;
    public static double p = 0.003, i = 0, d = 0.00015;
    public static double f = 0.08;
    public final double ticks_per_rev = 537.6;
    boolean isPropDetected = false;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // leftClaw = hardwareMap.get(ServoImplEx.class, "leftClaw");
        //rightClaw = hardwareMap.get(ServoImplEx.class, "rightClaw"); //wrist
        armmotor = hardwareMap.get(DcMotorEx.class, "armmotor");
        armmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        controller = new PIDController(p, i, d);
        telemetry.addData("posleftslide", leftSlide.getCurrentPosition());
        telemetry.addData("posrightslide", rightSlide.getCurrentPosition());
        telemetry.addData("posarmmotor", armmotor.getCurrentPosition());
        telemetry.update();
        RightClawSubsystem.initialize();

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipelineBlue myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipelineBlue(borderLeftX, borderRightX, borderTopY, borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }


            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        FtcDashboard dashboard = FtcDashboard.getInstance(); //this all the way to telemetry.update(); is available at 192.168.43.1:8080/dash
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 30);
        telemetry.update();

        /* Trajectories either consist of vectors or poses. Vectors are for moving only x and y coordinates while poses have a heading(angle)
            For example
            a pose at coordinates (10,-10) facing 120 degrees would look like
            Pose2d myPose = new Pose2d(10,-10, Math.toRadians(120));
            Assuming you start at (0,0) at the start of the program, the robot with move to the coordinates labeled at an 120 degree heading
         */

        waitForStart();

        if(opModeIsActive()) {
            telemetry.addData("posleftslide", leftSlide.getCurrentPosition());
            telemetry.addData("posrightslide", rightSlide.getCurrentPosition());
            telemetry.addData("posarmmotor", armmotor.getCurrentPosition());
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading", drive.getPoseEstimate().getHeading());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }


            Pose2d startPoseBlue = new Pose2d(-36.2, 61, Math.toRadians(270));

            drive.setPoseEstimate(startPoseBlue);

            int pixelArmAngle = -2450;
            int backBoardAngle = -500;
            int slideCenter = 4700;
            int slideRight = 4500;
            int slideLeft = 4500;
            int slideStartPos = 50;
            int slideBackBoard = 4700;


            TrajectorySequence rightTapeParkLeft = drive.trajectorySequenceBuilder(startPoseBlue)

                    .addTemporalMarker(() -> {
                        RightClawSubsystem.armPosition(pixelArmAngle);
                    })
                    .lineToConstantHeading(new Vector2d(-46.5,55))
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideRight);
                    })
                    .waitSeconds(3)
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideStartPos);
                    })
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.armPosition(backBoardAngle);
                    })
                    .waitSeconds(0.5)
                    .splineToSplineHeading(new Pose2d(-43.2,45, Math.toRadians(270)), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(-33.2,22, Math.toRadians(270)), Math.toRadians(280))
                    .splineToSplineHeading(new Pose2d(10,5, Math.toRadians(0)), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(46.5,28, Math.toRadians(0)), Math.toRadians(0))
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideBackBoard);
                    })
                    .waitSeconds(3)
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideStartPos);
                    })
                    .waitSeconds(.6)
                    .lineToConstantHeading(new Vector2d(46.5, 60))
                    .lineToConstantHeading(new Vector2d(60, 60))
                    .build();


            TrajectorySequence rightTapeParkRight = drive.trajectorySequenceBuilder(startPoseBlue)


                    .addTemporalMarker(() -> {
                        RightClawSubsystem.armPosition(pixelArmAngle);
                    })
                    .lineToConstantHeading(new Vector2d(-46.5,55))
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideRight);
                    })
                    .waitSeconds(3)
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideStartPos);
                    })
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.armPosition(backBoardAngle);
                    })
                    .waitSeconds(0.5)
                    .splineToConstantHeading(new Vector2d(-43.2,45), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-33.2,12), Math.toRadians(280))
                    .splineToSplineHeading(new Pose2d(10,5, Math.toRadians(270)), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(46.5,28, Math.toRadians(0)), Math.toRadians(0))
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideBackBoard);
                    })
                    .waitSeconds(3)
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideStartPos);
                    })
                    .waitSeconds(.6)
                    .lineToConstantHeading(new Vector2d(46.5, 12))
                    .lineToConstantHeading(new Vector2d(60, 12))
                    .build();


        TrajectorySequence centerTapeParkLeft = drive.trajectorySequenceBuilder(startPoseBlue)


                    .addTemporalMarker(() -> {
                        RightClawSubsystem.armPosition(pixelArmAngle);
                    })
                .lineToLinearHeading(new Pose2d(-47.5,38, Math.toRadians(295)))
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideCenter);
                    })
                    .waitSeconds(3)
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideStartPos);
                    })
                .waitSeconds(0.5)
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.armPosition(backBoardAngle);
                    })
                    .waitSeconds(0.5)
                    .splineToSplineHeading(new Pose2d(-50.2,20, Math.toRadians(295)), Math.toRadians(280))
                    .splineToSplineHeading(new Pose2d(36,32, Math.toRadians(340)), Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(46.5,34, Math.toRadians(0)), Math.toRadians(0))
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideBackBoard);
                    })
                    .waitSeconds(3)
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideStartPos);
                    })
                    .waitSeconds(.6)
                    .lineToConstantHeading(new Vector2d(49.5, 60))
                    .lineToConstantHeading(new Vector2d(60, 60))
                    .build();


            TrajectorySequence centerTapeParkRight = drive.trajectorySequenceBuilder(startPoseBlue)//(0,0) is the starting position and 270 degrees is the direction it is facing if you put it on a coordinate system(straight down)


                    .addTemporalMarker(() -> {
                        RightClawSubsystem.armPosition(pixelArmAngle);
                    })
                    .lineToLinearHeading(new Pose2d(-47.5,38, Math.toRadians(295)))
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideCenter);
                    })
                    .waitSeconds(3)
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideStartPos);
                    })
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.armPosition(backBoardAngle);
                    })
                    .waitSeconds(0.5)
                    .splineToSplineHeading(new Pose2d(-50.2,20, Math.toRadians(295)), Math.toRadians(280))
                    .splineToSplineHeading(new Pose2d(36,32, Math.toRadians(340)), Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(46.5,34, Math.toRadians(0)), Math.toRadians(0))
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideBackBoard);
                    })
                    .waitSeconds(3)
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideStartPos);
                    })
                    .waitSeconds(.6)
                    .lineToConstantHeading(new Vector2d(46.5, 12))
                    .lineToConstantHeading(new Vector2d(60, 12))
                    .build();


            TrajectorySequence leftTapeLeft = drive.trajectorySequenceBuilder(startPoseBlue)


                    .addTemporalMarker(() -> {
                        RightClawSubsystem.armPosition(pixelArmAngle);
                    })
                    .lineToLinearHeading(new Pose2d(-36.2,47, Math.toRadians(310)))
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideLeft);
                    })
                    .waitSeconds(3)
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideStartPos);
                    })
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.armPosition(backBoardAngle);
                    })
                    .waitSeconds(1)
                    .splineToSplineHeading(new Pose2d(-40.2,20, Math.toRadians(295)), Math.toRadians(280))
                    .splineToSplineHeading(new Pose2d(36,28.9, Math.toRadians(270)), Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(46.5,42, Math.toRadians(0)), Math.toRadians(0))
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideBackBoard);
                    })
                    .waitSeconds(3)
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideStartPos);
                    })
                    .waitSeconds(.6)
                    .lineToConstantHeading(new Vector2d(46.5, 60))
                    .lineToConstantHeading(new Vector2d(54, 60))
                    .build();


            TrajectorySequence leftTapeParkRight = drive.trajectorySequenceBuilder(startPoseBlue)


                    .addTemporalMarker(() -> {
                        RightClawSubsystem.armPosition(pixelArmAngle);
                    })
                    .lineToLinearHeading(new Pose2d(-36.2,47, Math.toRadians(310)))
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideLeft);
                    })
                    .waitSeconds(3)
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideStartPos);
                    })
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.armPosition(backBoardAngle);
                    })
                    .waitSeconds(1)
                    .splineToSplineHeading(new Pose2d(-40.2,20, Math.toRadians(295)), Math.toRadians(280))
                    .splineToSplineHeading(new Pose2d(36,28.9, Math.toRadians(270)), Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(46.5,42, Math.toRadians(0)), Math.toRadians(0))
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideBackBoard);
                    })
                    .waitSeconds(3)
                    .addTemporalMarker(() -> {
                        RightClawSubsystem.syncedSlides(slideStartPos);
                    })
                    .waitSeconds(.6)
                    .lineToConstantHeading(new Vector2d(46.5, 12))
                    .lineToConstantHeading(new Vector2d(54, 12))
                    .build();


            if (myPipeline.getRectArea() > 2000) {
                if (myPipeline.getRectMidpointX() > 240) {

                    telemetry.addLine("Autonomous Center");
                    webcam.stopStreaming();
                    drive.followTrajectorySequence(centerTapeParkRight);

                } else if (myPipeline.getRectMidpointX() > 10) {

                    telemetry.addLine("Autonomous Left");
                    webcam.stopStreaming();
                    drive.followTrajectorySequence(leftTapeParkRight);

                }
            }
        if (myPipeline.getRectArea() < 2000) {

            if(!isPropDetected){

                webcam.stopRecordingPipeline();
                webcam.stopStreaming();

            }

            telemetry.addLine("Autonomous Right");
            webcam.stopStreaming();
            drive.followTrajectorySequence(rightTapeParkRight);

        }


            isStopRequested();

        }
    }
