package Competition;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.Vision.ContourPipelineRed;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "AutoRedBoard", group = "Competition")
public class AutoRedBoard extends LinearOpMode {
    private static ServoImplEx leftClaw;
    private static ServoImplEx rightClaw;
    private static DcMotor armmotor;
    private static final int CAMERA_WIDTH  = 640; // width  of camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of camera resolution
    public static double borderLeftX    = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX   = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY     = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY  = 0.0;   //fraction of pixels from the bottom of the cam to skip
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 188.0, 60.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 100.0);
    private OpenCvCamera webcam;
    TrajectorySequence leftTape;
    TrajectorySequence middleTape;
    TrajectorySequence rightTape;
    public static DcMotorEx leftSlide;
    public static DcMotorEx rightSlide;
    public static PIDController controller;
    public static double p=0.003, i=0, d=0.00015;
    public static double f = 0.08;
    public final double ticks_per_rev = 537.6;
    public int startingPosArm = -300;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
       // leftClaw = hardwareMap.get(ServoImplEx.class, "leftClaw");
        //rightClaw = hardwareMap.get(ServoImplEx.class, "rightClaw"); //wrist
        armmotor = hardwareMap.get(DcMotor.class, "armmotor");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        controller = new PIDController(p, i, d);
        if (armmotor.getCurrentPosition() >= -5) { //sets the arms starting position using pid
            armmotor.setTargetPosition(startingPosArm);
            armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            controller.setPID(p, i, d);
            int slidePos = armmotor.getCurrentPosition();
            double pid = controller.calculate(slidePos, startingPosArm);
            double ff = Math.cos(Math.toRadians(startingPosArm / ticks_per_rev)) * f;

            double power = pid + ff;
            armmotor.setPower(power);
        }


        int s = 1;
        int startpos = 50;
        int rightfullyex = -2000;
        int leftfullyex = 2000;
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipelineRed myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipelineRed(borderLeftX,borderRightX,borderTopY,borderBottomY));
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



        Pose2d startPoseRedBB = new Pose2d(16, -61, Math.toRadians(90));
        Pose2d startPoseBlueBB = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d startPoseBlue = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d startPoseRed = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPoseRedBB);

        TrajectorySequence genesis = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))//(0,0) is the starting position and 270 degrees is the direction it is facing if you put it on a coordinate system(straight down)
                .lineToLinearHeading(new Pose2d(10,-35,Math.toRadians(270)))
                .lineTo(new Vector2d(22,-35))
                .build();
        TrajectorySequence wowie = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))//(0,0) is the starting position and 270 degrees is the direction it is facing if you put it on a coordinate system(straight down)
                .lineToLinearHeading(new Pose2d(10,-35,Math.toRadians(270)))
                .build();

        TrajectorySequence dome = drive.trajectorySequenceBuilder(startPoseRedBB)
                .lineToLinearHeading(new Pose2d(49.5,-61,Math.toRadians(0)))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        if(myPipeline.getRectArea() > 2000){
            if(myPipeline.getRectMidpointX() > 400){
                telemetry.addLine("Autonomous Right");
                telemetry.update();
                webcam.stopStreaming();
                drive.followTrajectorySequence(genesis);

            } else if(myPipeline.getRectMidpointX() > 200){
                telemetry.addLine("Autonomous Center");
                telemetry.update();
                webcam.stopStreaming();
                drive.followTrajectorySequence(wowie);

            } else {
                telemetry.addLine("Autonomous Left");
                telemetry.update();
                webcam.stopStreaming();
                drive.followTrajectorySequence(dome);
            }
        }
        waitForStart();

        isStopRequested();
    }
}