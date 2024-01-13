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

import static org.firstinspires.ftc.teamcode.hardwareinit.armmotor;
import static org.firstinspires.ftc.teamcode.hardwareinit.leftSlide;
import static org.firstinspires.ftc.teamcode.hardwareinit.rightSlide;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems;
import org.firstinspires.ftc.teamcode.common.Vision.ContourPipelineBlue;
import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "AutoBlueBoard", group = "org/firstinspires/ftc/teamcode/drive/opmode/Competition")
public class AutoBlueBoard extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ContourPipelineBlue myPipeline;

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
        Subsystems.initialize();

        // OpenCV webcam

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        myPipeline = new ContourPipelineBlue(borderLeftX, borderRightX, borderTopY, borderBottomY);
        webcam.setPipeline(myPipeline);
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

        Pose2d startPoseBlueBB = new Pose2d(16, 61, Math.toRadians(270));

        drive.setPoseEstimate(startPoseBlueBB);

        int pixelArmAngle = -2450;
        int backBoardAngle = -500;
        int slideCenter = 4700;
        int slideRight = 4500;
        int slideLeft = 3000;
        int slideStartPos = 50;
        int slideBackBoard = 4700;


        TrajectorySequence rightTapeParkLeft = drive.trajectorySequenceBuilder(startPoseBlueBB)


                .addTemporalMarker(() -> {
                    Subsystems.armPosition(pixelArmAngle);
                })
                .lineToLinearHeading(new Pose2d(20, 43, Math.toRadians(203)))
                .addTemporalMarker(() -> {
                    Subsystems.syncedSlides(slideRight);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    Subsystems.armPosition(backBoardAngle);
                })
                .lineToLinearHeading(new Pose2d(48.5, 28.9, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    Subsystems.syncedSlides(slideBackBoard);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    Subsystems.syncedSlides(slideStartPos);
                })
                .waitSeconds(.6)
                .lineToConstantHeading(new Vector2d(49.5, 60))
                .lineToConstantHeading(new Vector2d(60, 60))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(14,61,Math.toRadians(270)), Math.toRadians(180))
                .build();


        TrajectorySequence rightTapeParkRight = drive.trajectorySequenceBuilder(startPoseBlueBB)


                .addTemporalMarker(() -> {
                    Subsystems.armPosition(pixelArmAngle);
                })
                .lineToLinearHeading(new Pose2d(20, 43, Math.toRadians(203)))
                .addTemporalMarker(() -> {
                    Subsystems.syncedSlides(slideRight);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    Subsystems.armPosition(backBoardAngle);
                })
                .lineToLinearHeading(new Pose2d(48.5, 28.9, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    Subsystems.syncedSlides(slideBackBoard);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    Subsystems.syncedSlides(slideStartPos);
                })
                .waitSeconds(.6)
                .lineToConstantHeading(new Vector2d(49.5, 12))
                .lineToConstantHeading(new Vector2d(60, 12))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(14,61,Math.toRadians(270)), Math.toRadians(180))
                .build();


        TrajectorySequence centerTapeParkLeft = drive.trajectorySequenceBuilder(startPoseBlueBB)


                .addTemporalMarker(() -> {
                    Subsystems.armPosition(pixelArmAngle);
                })
                .lineToLinearHeading(new Pose2d(32, 38, Math.toRadians(245)))
                .addTemporalMarker(() -> {
                    Subsystems.syncedSlides(slideCenter);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    Subsystems.armPosition(backBoardAngle);
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(48.5, 32.8, Math.toRadians(4)))
                .addTemporalMarker(() -> {
                    Subsystems.syncedSlides(slideBackBoard);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    Subsystems.syncedSlides(slideStartPos);
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(49.5, 60))
                .lineToConstantHeading(new Vector2d(60, 60))
                .build();


        TrajectorySequence centerTapeParkRight = drive.trajectorySequenceBuilder(startPoseBlueBB)//(0,0) is the starting position and 270 degrees is the direction it is facing if you put it on a coordinate system(straight down)


                .addTemporalMarker(() -> {
                    Subsystems.armPosition(pixelArmAngle);
                })
                .lineToLinearHeading(new Pose2d(33, 40, Math.toRadians(240)))
                .addTemporalMarker(() -> {
                    Subsystems.syncedSlides(slideCenter);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    Subsystems.armPosition(backBoardAngle);
                })
                .lineToLinearHeading(new Pose2d(49.5, 32.8, Math.toRadians(4)))
                .addTemporalMarker(() -> {
                    Subsystems.syncedSlides(slideBackBoard);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    Subsystems.syncedSlides(slideStartPos);
                })
                .waitSeconds(0.6)
                .lineToConstantHeading(new Vector2d(49.5, 12))
                .lineToConstantHeading(new Vector2d(60, 12))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(14,61,Math.toRadians(270)), Math.toRadians(180))
                .build();


        TrajectorySequence leftTapeLeft = drive.trajectorySequenceBuilder(startPoseBlueBB)


                .addTemporalMarker(() -> {
                    Subsystems.armPosition(pixelArmAngle);
                })
                .lineToConstantHeading(new Vector2d(28,53))
                .addTemporalMarker(() -> {
                    Subsystems.syncedSlides(slideLeft);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    Subsystems.armPosition(backBoardAngle);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(49.5, 39.1, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    Subsystems.syncedSlides(slideBackBoard);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    Subsystems.syncedSlides(slideStartPos);
                })
                .waitSeconds(.3)
                .lineToConstantHeading(new Vector2d(49.5, 60))
                .lineToConstantHeading(new Vector2d(60, 60))
                .build();


        TrajectorySequence leftTapeParkRight = drive.trajectorySequenceBuilder(startPoseBlueBB)


                .addTemporalMarker(() -> {
                    Subsystems.armPosition(pixelArmAngle);
                })
                .lineToConstantHeading(new Vector2d(28,53))
                .addTemporalMarker(() -> {
                    Subsystems.syncedSlides(slideLeft);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    Subsystems.armPosition(backBoardAngle);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(49.5, 39.1, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    Subsystems.syncedSlides(slideBackBoard);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    Subsystems.syncedSlides(slideStartPos);
                })
                .waitSeconds(.7)
                .lineToConstantHeading(new Vector2d(49.5, 12))
                .lineToConstantHeading(new Vector2d(60, 12))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(14,61,Math.toRadians(270)), Math.toRadians(180))
                .build();



        if (myPipeline.getRectArea() > 2000) {
                if (myPipeline.getRectMidpointX() > 240) {
                    telemetry.addLine("Autonomous Center");
                    webcam.stopStreaming();
                    drive.followTrajectorySequence(centerTapeParkRight);
                    telemetry.addData("x", drive.getPoseEstimate().getX());
                    telemetry.addData("y", drive.getPoseEstimate().getY());
                    telemetry.addData("heading", drive.getPoseEstimate().getHeading());
                    telemetry.update();

                } else if (myPipeline.getRectMidpointX() > 10) {
                    telemetry.addLine("Autonomous Left");
                    webcam.stopStreaming();
                    drive.followTrajectorySequence(leftTapeParkRight);
                    telemetry.addData("x", drive.getPoseEstimate().getX());
                    telemetry.addData("y", drive.getPoseEstimate().getY());
                    telemetry.addData("heading", drive.getPoseEstimate().getHeading());
                    telemetry.update();

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
                    telemetry.addData("x", drive.getPoseEstimate().getX());
                    telemetry.addData("y", drive.getPoseEstimate().getY());
                    telemetry.addData("heading", drive.getPoseEstimate().getHeading());
                    telemetry.update();
            }

            isStopRequested();

        }
    }
