package Competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ServoImplEx;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsytems;
import org.firstinspires.ftc.teamcode.Vision.ContourPipelineBlue;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Vector;

@Autonomous(name = "AutoBlueBoard", group = "Competition")
public class AutoBlueBoard extends LinearOpMode {
    private static ServoImplEx leftClaw;
    private static ServoImplEx rightClaw;
    private static DcMotorEx armmotor;
    private static final int CAMERA_WIDTH  = 640; // width  of camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of camera resolution
    public static double borderLeftX    = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX   = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY     = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY  = 0.0;   //fraction of pixels from the bottom of the cam to skip
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 188.0, 60.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 100.0);
    private OpenCvCamera webcam;
    public static DcMotorEx leftSlide;
    public static DcMotorEx rightSlide;
    public static PIDController controller;
    public static double p=0.003, i=0, d=0.00015;
    public static double f = 0.08;
    public final double ticks_per_rev = 537.6;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // leftClaw = hardwareMap.get(ServoImplEx.class, "leftClaw");
        //rightClaw = hardwareMap.get(ServoImplEx.class, "rightClaw"); //wrist
        armmotor = hardwareMap.get(DcMotorEx.class, "armmotor");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        controller = new PIDController(p, i, d);
        telemetry.addData("posleftslide", leftSlide.getCurrentPosition());
        telemetry.addData("posrightslide", rightSlide.getCurrentPosition());
        telemetry.addData("posarmmotor", armmotor.getCurrentPosition());
        telemetry.update();

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipelineBlue myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipelineBlue(borderLeftX,borderRightX,borderTopY,borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }


            @Override
            public void onError(int errorCode)
            {
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



        Pose2d startPoseBlueBB = new Pose2d(16, 61, Math.toRadians(270));

        drive.setPoseEstimate(startPoseBlueBB);

        int pixelArmAngle = -3500;
        int backBoardAngle = -300;
        int slideFullEx = 5700;
        int slideStartPos = 50;


        TrajectorySequence rightTapeParkLeft = drive.trajectorySequenceBuilder(startPoseBlueBB)
                .addTemporalMarker(() -> {
                    Subsytems.armPosition(pixelArmAngle);
                })
                .lineToLinearHeading(new Pose2d(16, 32.8, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideFullEx);
                })
                .waitSeconds(5)
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideStartPos);
                })
                .addTemporalMarker(() -> {
                    Subsytems.armPosition(backBoardAngle);
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(49.5,24.9, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideFullEx);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideStartPos);
                })
                .waitSeconds(.3)
                .lineToConstantHeading(new Vector2d(49.5,60))
                .lineToConstantHeading(new Vector2d(54,60))
                .build();

        TrajectorySequence rightTapeParkRight = drive.trajectorySequenceBuilder(startPoseBlueBB)
                .addTemporalMarker(() -> {
                    Subsytems.armPosition(pixelArmAngle);
                })
                .lineToLinearHeading(new Pose2d(16, 32.8, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideFullEx);
                })
                .waitSeconds(5)
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideStartPos);
                })
                .addTemporalMarker(() -> {
                    Subsytems.armPosition(backBoardAngle);
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(49.5,24.9, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideFullEx);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideStartPos);
                })
                .waitSeconds(.3)
                .lineToConstantHeading(new Vector2d(49.5,12))
                .lineToConstantHeading(new Vector2d(54,12))
                .build();
        TrajectorySequence centerTapeParkLeft = drive.trajectorySequenceBuilder(startPoseBlueBB)
                .addTemporalMarker(() -> {
                    Subsytems.armPosition(pixelArmAngle);
                })
                .lineToConstantHeading(new Vector2d(16,32.8))
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideFullEx);
                })
                .waitSeconds(5)
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideStartPos);
                })
                .addTemporalMarker(() -> {
                    Subsytems.armPosition(backBoardAngle);
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(49.5,32.8, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideFullEx);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideStartPos);
                })
                .waitSeconds(.3)
                .lineToConstantHeading(new Vector2d(49.5,60))
                .lineToConstantHeading(new Vector2d(54,60))
                .build();
        TrajectorySequence centerTapeParkRight = drive.trajectorySequenceBuilder(startPoseBlueBB)//(0,0) is the starting position and 270 degrees is the direction it is facing if you put it on a coordinate system(straight down)
                .addTemporalMarker(() -> {
                    Subsytems.armPosition(pixelArmAngle);
                })
                .lineToConstantHeading(new Vector2d(16,32.8))
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideFullEx);
                })
                .waitSeconds(5)
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideStartPos);
                })
                .addTemporalMarker(() -> {
                    Subsytems.armPosition(backBoardAngle);
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(49.5,32.8, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideFullEx);
                })
                .waitSeconds(3)
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideStartPos);
                })
                .waitSeconds(.3)
                .lineToConstantHeading(new Vector2d(49.5,12))
                .lineToConstantHeading(new Vector2d(54,12))
                .build();
        TrajectorySequence leftTapeLeft = drive.trajectorySequenceBuilder(startPoseBlueBB)
                .addTemporalMarker(() -> {
                    Subsytems.armPosition(pixelArmAngle);
                })
                .lineToLinearHeading(new Pose2d(16,32.8, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideFullEx);
                })
                .waitSeconds(5)
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideStartPos);
                })
                .addTemporalMarker(() -> {
                    Subsytems.armPosition(backBoardAngle);
                })
                .waitSeconds(.5)
                .lineToConstantHeading(new Vector2d(16,50))
                .splineToSplineHeading(new Pose2d(49.5, 39.1, Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideFullEx);
                })
                .waitSeconds(5)
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideStartPos);
                })
                .waitSeconds(.3)
                .lineToConstantHeading(new Vector2d(49.5,60))
                .lineToConstantHeading(new Vector2d(54,60))
                .build();
        TrajectorySequence leftTapeParkRight = drive.trajectorySequenceBuilder(startPoseBlueBB)
                .addTemporalMarker(() -> {
                    Subsytems.armPosition(pixelArmAngle);
                })
                .lineToLinearHeading(new Pose2d(16,32.8, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideFullEx);
                })
                .waitSeconds(5)
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideStartPos);
                })
                .addTemporalMarker(() -> {
                    Subsytems.armPosition(backBoardAngle);
                })
                .waitSeconds(.5)
                .lineToConstantHeading(new Vector2d(16,50))
                .splineToSplineHeading(new Pose2d(49.5, 39.1, Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideFullEx);
                })
                .waitSeconds(5)
                .addTemporalMarker(() -> {
                    Subsytems.syncedSlides(slideStartPos);
                })
                .waitSeconds(.3)
                .lineToConstantHeading(new Vector2d(49.5,12))
                .lineToConstantHeading(new Vector2d(54,12))
                .build();



        if(myPipeline.getRectArea() > 2000){
            if(myPipeline.getRectMidpointX() > 420){
                telemetry.addLine("Autonomous Right");
                telemetry.update();
                webcam.stopStreaming();
                drive.followTrajectorySequence(rightTapeParkRight);
                drive.isBusy();
                telemetry.addData("x" , drive.getPoseEstimate().getX());
                telemetry.addData("y" , drive.getPoseEstimate().getY());
                telemetry.addData("heading" , drive.getPoseEstimate().getHeading());
                telemetry.update();

            } else if(myPipeline.getRectMidpointX() >= 210){
                telemetry.addLine("Autonomous Center");
                telemetry.update();
                webcam.stopStreaming();
                drive.followTrajectorySequence(centerTapeParkRight);
                drive.isBusy();
                telemetry.addData("x" , drive.getPoseEstimate().getX());
                telemetry.addData("y" , drive.getPoseEstimate().getY());
                telemetry.addData("heading" , drive.getPoseEstimate().getHeading());
                telemetry.update();

            } else if (myPipeline.getRectMidpointX() < 210){
                telemetry.addLine("Autonomous Left");
                telemetry.update();
                webcam.stopStreaming();
                drive.followTrajectorySequence(leftTapeParkRight);
                drive.isBusy();
                telemetry.addData("x" , drive.getPoseEstimate().getX());
                telemetry.addData("y" , drive.getPoseEstimate().getY());
                telemetry.addData("heading" , drive.getPoseEstimate().getHeading());
                telemetry.update();

            } else {
                telemetry.addLine("Autonomous Left");
                telemetry.update();
                webcam.stopStreaming();
                drive.followTrajectorySequence(leftTapeParkRight);
                drive.isBusy();
                telemetry.addData("x" , drive.getPoseEstimate().getX());
                telemetry.addData("y" , drive.getPoseEstimate().getY());
                telemetry.addData("heading" , drive.getPoseEstimate().getHeading());
                telemetry.update();
            }
        }

        waitForStart();

        isStopRequested();

    }
}