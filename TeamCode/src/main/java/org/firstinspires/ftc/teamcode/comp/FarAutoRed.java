package org.firstinspires.ftc.teamcode.comp;

import static org.firstinspires.ftc.teamcode.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.Robot.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems;
import org.firstinspires.ftc.teamcode.Vision.ContourPipelineBlue;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "FarAutoRed", group = "Competition")
public class FarAutoRed extends LinearOpMode {

    private enum State {
        INITIAL,
        BACKBOARD,
        PARK,
        TAPE_CAMERA,
        RIGHT_TAPE,
        LEFT_TAPE,
        MIDDLE_TAPE,
        SLIDE_EXTENSION,
        RIGHT_CLAW_OPEN,
        TRANSITION_POS,
        DELAY_START,
        LEFT_CLAW_OPEN,
        SLIDE_RETRACT, GOING_TO_BACKBOARD, DONE,

    }

    private enum Board {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 188.0, 60.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 100.0);

    public final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime StateTime = new ElapsedTime();
    ElapsedTime delayTimer = new ElapsedTime();
    boolean delayStart = true; // Default: no delay
    double selectedDelayTime = 5.0; // Default delay time in seconds
    private boolean leftPark = false; // Default: right park

    private static final int CAMERA_WIDTH = 640; // width  of camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of camera resolution
    public static double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip
    /*
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 180.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 96, 255);
    */
    private OpenCvCamera webcam;
    public static PIDController controller;
    public static double p = 0.003, i = 0, d = 0.00015;
    public static double f = 0.08;
    public final double ticks_per_rev = 537.6;
    private static final int PIXEL_ARM_ANGLE = 3125;
    private static final int BACK_BOARD_ANGLE = 2400;
    private static final int ARM_RESTING_POSITION = 0;
    private static final int ARM_UNDER_BAR = 2200;
    private static final int SLIDE_EXTENDED = 1800;
    private static final int SLIDE_START_POS = 0;
    private static final int SLIDE_BACKBOARD = 1050;
    private static final int SLIDE_RIGHT_TAPE = 1200;
    private static final int PIXEL_STACK_ANGLE = 3000;
    private static final int PIXEL_STACK_EXTENSION = 1300;
    private static final double LEFT_CLAW_OPEN = 0.54;
    private static final double LEFT_CLAW_CLOSE = 0.17;
    private static final double RIGHT_CLAW_OPEN = 0.2;
    private static final double RIGHT_CLAW_CLOSE = 0.6;
    private static final double WRIST_PIXEL_PICKUP = 0.46;
    private static final double WRIST_BACKBOARD = 0.73;
    private static final double WRIST_DOWN = 0;
    private static final double WRIST_UP = 1;
    private static final double WRIST_LEFT_TAPE = 0.5;
    static boolean pressed = false;

    Pose2d startPoseRed = new Pose2d(-36.2, -61, Math.toRadians(90));


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        // Slide Motors
        slideExtension = hardwareMap.get(DcMotorEx.class, "slideExtension");
        slideExtension.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slideExtension.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slidePivot = hardwareMap.get(DcMotorEx.class, "slidePivot");
        slidePivot.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slidePivot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Servos
        wrist = hardwareMap.get(Servo.class, "wrist");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        rightClaw.setPosition(0.6);
        leftClaw.setPosition(0.17);
        // Touch Sensor
        touch = hardwareMap.get(TouchSensor.class, "touch");

        drive.setPoseEstimate(startPoseRed);

        Subsystems.init();

        // OpenCV webcam

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipelineBlue myPipeline = new ContourPipelineBlue(borderLeftX, borderRightX, borderTopY, borderBottomY);
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


        // Allow the user to select the delay time using the controller



        while (!opModeIsActive()) {
            telemetry.addData("Delay Start", delayStart ? "Yes" : "No");
            telemetry.addData("Selected Delay Time", "%.1f seconds", selectedDelayTime);
            telemetry.addData("Press B to toggle park side", "Current: " + (leftPark ? "Left" : "Right"));
            telemetry.update();
            // Allow the user to select the delay time using the controller
            if (gamepad1.a) {
                delayStart = !delayStart;
            }
            if (delayStart) {
                if (gamepad1.dpad_up) {
                    selectedDelayTime += 1.0;
                } else if (gamepad1.dpad_down && selectedDelayTime > 1.0) {
                    selectedDelayTime -= 1.0;
                }

            }
            if (gamepad1.b) {
                leftPark = !leftPark;
            }
        }

        /* Trajectories either consist of vectors or poses. Vectors are for moving only x and y coordinates while poses have a heading(angle)
            For example
            a pose at coordinates (10,-10) facing 120 degrees would look like
            Pose2d myPose = new Pose2d(10,-10, Math.toRadians(120));
            Assuming you start at (0,0) at the start of the program, the robot with move to the coordinates labeled at an 120 degree heading
         */
        TrajectorySequence leftTape = drive.trajectorySequenceBuilder(startPoseRed)

                .lineToConstantHeading(new Vector2d(-49.2, -59))
                .build();

        TrajectorySequence middleTape = drive.trajectorySequenceBuilder(startPoseRed)

                .lineToLinearHeading(new Pose2d(-44.1, -53.4, Math.toRadians(75)))
                .build();

        TrajectorySequence rightTape = drive.trajectorySequenceBuilder(startPoseRed)

                .lineToLinearHeading(new Pose2d(-47.6, -48, Math.toRadians(35)))
                .build();

        // white pixel stacks (R,L, & M indicating starting point, all same end location):

        TrajectorySequence whiteStack = drive.trajectorySequenceBuilder(currentPose)
                .lineToLinearHeading(new Pose2d(-51.5,-32, Math.toRadians(180)))
                .build();

        // transition moves (to get to position before crossing mid):

        TrajectorySequence crossMap = drive.trajectorySequenceBuilder(currentPose)
                .lineToLinearHeading(new Pose2d(40, -2, Math.toRadians(0)))
                .build();
        TrajectorySequence poseToCross = drive.trajectorySequenceBuilder(currentPose)
                //.lineToLinearHeading(new Pose2d(-52, 12, Math.toRadians(33)))
                .lineToLinearHeading(new Pose2d(-47.6, -2, Math.toRadians(0)))
                .addDisplacementMarker(() -> drive.followTrajectorySequenceAsync(crossMap))
                .build();



        // backboard locations:

        TrajectorySequence backBoardLeft = drive.trajectorySequenceBuilder(rightTape.end())

                .lineToLinearHeading(new Pose2d(49.4, -25.9, Math.toRadians(0)))
                .build();

        TrajectorySequence backBoardMiddle = drive.trajectorySequenceBuilder(middleTape.end())

                .lineToLinearHeading(new Pose2d(49.4, -31.8, Math.toRadians(0)))
                .build();

        TrajectorySequence backBoardRight = drive.trajectorySequenceBuilder(leftTape.end())

                .lineToLinearHeading(new Pose2d(49.4, -36.1, Math.toRadians(0)))
                .build();



        // parking spots:

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(currentPose)

                .lineToConstantHeading(new Vector2d(51, -60))
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(currentPose)

                .lineToConstantHeading(new Vector2d(51, -8))
                .build();

        waitForStart();

        if (opModeIsActive()) {

            runtime.reset();
            StateTime.reset();
            delayTimer.reset();
            Board board = Board.LEFT;
            State state;
            if (delayStart) {
                state = State.DELAY_START;
            } else {
                state = State.INITIAL;
            }

            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("CurrentState", state.toString());
                telemetry.addData("CurrentBoard", board.toString());
                telemetry.addData("Status", "Run Time: " + runtime);
                telemetry.addData("posslide", slideExtension.getCurrentPosition());
                telemetry.addData("posarmPivot", slidePivot.getCurrentPosition());
                telemetry.addData("x", drive.getPoseEstimate().getX());
                telemetry.addData("y", drive.getPoseEstimate().getY());
                telemetry.addData("heading", drive.getPoseEstimate().getHeading());
                telemetry.addData("drive", drive.isBusy());
                telemetry.update();
                if (touch.isPressed() && !pressed) {

                    telemetry.addData("Touch Sensor", "Is Pressed");
                    telemetry.update();
                    if (slideExtension.getPower() <= 0) {
                        slideExtension.setPower(0);
                    }
                    slideExtension.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    pressed = true;

                }




                switch (state) {

                    case DELAY_START:

                        if (delayStart && delayTimer.seconds() >= selectedDelayTime) {
                            state = State.INITIAL;
                        } else if (!delayStart) {
                            state = State.INITIAL;
                        }
                        break;

                    case INITIAL:

                        wrist.setPosition(WRIST_PIXEL_PICKUP);
                        rightClaw.setPosition(RIGHT_CLAW_CLOSE);
                        leftClaw.setPosition(LEFT_CLAW_CLOSE);
                        state = State.TAPE_CAMERA;
                        break;

                    case TAPE_CAMERA:

                        if (myPipeline.getRectArea() > 2000) {
                            if (myPipeline.getRectMidpointX() < 275) {

                                webcam.stopStreaming();
                                drive.followTrajectorySequenceAsync(leftTape);
                                StateTime.reset();
                                state = State.LEFT_TAPE;

                            } else if (myPipeline.getRectMidpointX() > 275) {

                                webcam.stopStreaming();
                                drive.followTrajectorySequenceAsync(middleTape);
                                StateTime.reset();
                                state = State.MIDDLE_TAPE;

                            }
                        } else {

                            webcam.stopStreaming();
                            drive.followTrajectorySequenceAsync(rightTape);
                            StateTime.reset();
                            state = State.RIGHT_TAPE;

                        }

                        break;

                    case LEFT_TAPE:
                        Subsystems.slideAngle(PIXEL_ARM_ANGLE);

                        if (!drive.isBusy()) {

                            StateTime.reset();
                            state = State.SLIDE_EXTENSION;
                            board = Board.LEFT;

                        }

                        break;

                    case MIDDLE_TAPE:
                        Subsystems.slideAngle(PIXEL_ARM_ANGLE);

                        if (!drive.isBusy()) {

                            StateTime.reset();
                            state = State.SLIDE_EXTENSION;
                            board = Board.MIDDLE;

                        }

                        break;

                    case RIGHT_TAPE:
                        Subsystems.slideAngle(PIXEL_ARM_ANGLE);
                        wrist.setPosition(WRIST_LEFT_TAPE);

                        if (StateTime.time() >= 1) {

                            Subsystems.slideExtension(SLIDE_RIGHT_TAPE);

                        }

                        if (Math.abs(slideExtension.getCurrentPosition()) >= 1190) {

                            StateTime.reset();
                            state = State.RIGHT_CLAW_OPEN;
                            board = Board.RIGHT;

                        }

                        break;

                    case SLIDE_EXTENSION:

                        if (Math.abs(slidePivot.getCurrentPosition()) >= 2990) {
                            Subsystems.slideExtension(SLIDE_EXTENDED);
                        }

                        if (Math.abs(slideExtension.getCurrentPosition()) >= 1790) {

                            StateTime.reset();
                            state = State.RIGHT_CLAW_OPEN;

                        }

                        break;

                    case RIGHT_CLAW_OPEN:

                        rightClaw.setPosition(0.3);

                        if (StateTime.time() > 0.5) {

                            Subsystems.slideExtension(SLIDE_START_POS);
                            drive.followTrajectorySequenceAsync(poseToCross);
                            StateTime.reset();
                            state = State.GOING_TO_BACKBOARD;

                        }

                        break;


                    case GOING_TO_BACKBOARD:

                        Subsystems.slideAngle(ARM_RESTING_POSITION);
                        wrist.setPosition(WRIST_DOWN);


                        if (!drive.isBusy()) {

                            StateTime.reset();
                            state = State.BACKBOARD;

                        }

                        break;

                    case BACKBOARD:


                        if (board == Board.LEFT){

                            drive.followTrajectorySequence(backBoardLeft);

                        }
                        if (board == Board.MIDDLE){

                            drive.followTrajectorySequence(backBoardMiddle);

                        }
                        if (board == Board.RIGHT){

                            drive.followTrajectorySequence(backBoardRight);

                        }

                        if (!drive.isBusy()) {

                            StateTime.reset();
                            state = State.LEFT_CLAW_OPEN;

                        }

                        break;

                    case LEFT_CLAW_OPEN:
                        Subsystems.slideAngle(BACK_BOARD_ANGLE);
                        Subsystems.slideExtension(SLIDE_BACKBOARD);
                        wrist.setPosition(WRIST_BACKBOARD);


                        if (StateTime.time() > 1 ) {

                            leftClaw.setPosition(LEFT_CLAW_OPEN);
                            StateTime.reset();
                            state = State.SLIDE_RETRACT;

                        }

                        break;

                    case SLIDE_RETRACT:

                        Subsystems.slideExtension(SLIDE_START_POS);
                        Subsystems.slideAngle(ARM_RESTING_POSITION);
                        wrist.setPosition(WRIST_UP);

                        if (Math.abs(slideExtension.getCurrentPosition()) <= 20) {

                            StateTime.reset();
                            state = State.PARK;

                        }

                        break;

                    case PARK:


                        if (leftPark) {
                            drive.followTrajectorySequenceAsync(parkRight);
                        } else {

                            drive.followTrajectorySequenceAsync(parkLeft);

                        }
                        StateTime.reset();
                        state = State.DONE;

                        break;

                    case DONE:
                        break;

                    default:

                        StateTime.reset();
                        runtime.reset();

                }

                drive.update();

                currentPose = drive.getPoseEstimate();

            }
        }
    }
}