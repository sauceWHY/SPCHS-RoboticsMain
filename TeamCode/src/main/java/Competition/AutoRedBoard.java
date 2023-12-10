package Competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


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
    private static Servo hand;
    private static Servo arm;
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

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        hand = hardwareMap.get(Servo.class, "hand");
        arm = hardwareMap.get(Servo.class, "arm"); //wrist
        armmotor = hardwareMap.get(DcMotor.class, "armmotor");
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

        if(myPipeline.getRectArea() > 2000){
            if(myPipeline.getRectMidpointX() > 400){ //right side prop
                telemetry.addLine("Autonomous Right");
                telemetry.update();
                webcam.stopStreaming();
                drive.followTrajectorySequence(rightTape);

            } else if(myPipeline.getRectMidpointX() > 200){ //middle prop
                telemetry.addLine("Autonomous Center");
                telemetry.update();
                webcam.stopStreaming();
                drive.followTrajectorySequence(middleTape);

            } else { //left prop
                telemetry.addLine("Autonomous Left");
                telemetry.update();
                webcam.stopStreaming();
                drive.followTrajectorySequence(leftTape);
            }

        }

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

                leftTape = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(() -> hand.setPosition(.8))
                        .addTemporalMarker(() -> arm.setPosition(.8))
                        .lineToLinearHeading(new Pose2d(10,-35,Math.toRadians(270)))
                        .lineTo(new Vector2d(34,-37))
                        .waitSeconds(.5)
                        .addTemporalMarker(() -> {
                            armmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            armmotor.setTargetPosition(3500);
                            armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            armmotor.setPower(1);
                        }) //moves the arm to the drop the pixel on the backboard
                        .waitSeconds(2.5)
                        .addTemporalMarker(() -> arm.setPosition(.5)) //snaps the wrist to the front
                        .waitSeconds(.5)
                        .addTemporalMarker(() -> hand.setPosition(.2)) // opens the claw
                        .waitSeconds(5)
                        .build();
                middleTape = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(() -> hand.setPosition(.8))
                        .addTemporalMarker(() -> arm.setPosition(.8))
                        .lineToLinearHeading(new Pose2d(10,-35,Math.toRadians(270)))
                        .lineTo(new Vector2d(26,-35))
                        .waitSeconds(.5)
                        .addTemporalMarker(() -> {
                            armmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            armmotor.setTargetPosition(3500);
                            armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            armmotor.setPower(1);
                        }) //moves the arm to the drop the pixel on the backboard
                        .waitSeconds(2.5)
                        .addTemporalMarker(() -> arm.setPosition(.5)) //snaps the wrist to the front
                        .waitSeconds(.5)
                        .addTemporalMarker(() -> hand.setPosition(.2)) // opens the claw
                        .waitSeconds(5)
                        .build();
                rightTape = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(() -> hand.setPosition(.8))
                        .addTemporalMarker(() -> arm.setPosition(.8))
                        .lineToLinearHeading(new Pose2d(10,-35,Math.toRadians(270)))
                        .lineTo(new Vector2d(22,-35))
                        .waitSeconds(.5)
                        .addTemporalMarker(() -> {
                            armmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            armmotor.setTargetPosition(3500);
                            armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            armmotor.setPower(1);
                        }) //moves the arm to the drop the pixel on the backboard
                        .waitSeconds(2.5)
                        .addTemporalMarker(() -> arm.setPosition(.5)) //snaps the wrist to the front
                        .waitSeconds(.5)
                        .addTemporalMarker(() -> hand.setPosition(.2)) // opens the claw
                        .waitSeconds(5)
                        .build();







        waitForStart();

        if(isStopRequested()) return;






        /*TrajectorySequence genesis = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))//(0,0) is the starting position and 270 degrees is the direction it is facing if you put it on a coordinate system(straight down)
                .addTemporalMarker(() -> hand.setPosition(.8))
                .addTemporalMarker(() -> arm.setPosition(.8))
                .lineToLinearHeading(new Pose2d(10,-35,Math.toRadians(270)))
                .lineTo(new Vector2d(22,-35))
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    armmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armmotor.setTargetPosition(3500);
                    armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armmotor.setPower(1);
                }) //moves the arm to the drop the pixel on the backboard
                .waitSeconds(2.5)
                .addTemporalMarker(() -> arm.setPosition(.5)) //snaps the wrist to the front
                .waitSeconds(.5)
                .addTemporalMarker(() -> hand.setPosition(.2)) // opens the claw
                .waitSeconds(5)
                .build();
        TrajectorySequence wowie = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))//(0,0) is the starting position and 270 degrees is the direction it is facing if you put it on a coordinate system(straight down)
                .addTemporalMarker(() -> hand.setPosition(.8))
                .addTemporalMarker(() -> arm.setPosition(.8))
                .lineToLinearHeading(new Pose2d(10,-35,Math.toRadians(270)))
                .lineTo(new Vector2d(26,-35))
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    armmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armmotor.setTargetPosition(3500);
                    armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armmotor.setPower(1);
                }) //moves the arm to the drop the pixel on the backboard
                .waitSeconds(2.5)
                .addTemporalMarker(() -> arm.setPosition(.5)) //snaps the wrist to the front
                .waitSeconds(.5)
                .addTemporalMarker(() -> hand.setPosition(.2)) // opens the claw
                .waitSeconds(5)
                .build();
        TrajectorySequence dome = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .addTemporalMarker(() -> hand.setPosition(.8))
                .addTemporalMarker(() -> arm.setPosition(.8))
                .lineToLinearHeading(new Pose2d(10,-35,Math.toRadians(270)))
                .lineTo(new Vector2d(34,-37))
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    armmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armmotor.setTargetPosition(3500);
                    armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armmotor.setPower(1);
                }) //moves the arm to the drop the pixel on the backboard
                .waitSeconds(2.5)
                .addTemporalMarker(() -> arm.setPosition(.5)) //snaps the wrist to the front
                .waitSeconds(.5)
                .addTemporalMarker(() -> hand.setPosition(.2)) // opens the claw
                .waitSeconds(5)
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
        }*/

    }
}