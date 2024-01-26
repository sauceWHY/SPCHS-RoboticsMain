package org.firstinspires.ftc.teamcode.comp;

import static org.firstinspires.ftc.teamcode.Robot.leftClaw;
import static org.firstinspires.ftc.teamcode.Robot.rightClaw;
import static org.firstinspires.ftc.teamcode.Robot.slideExtension;
import static org.firstinspires.ftc.teamcode.Robot.slidePivot;
import static org.firstinspires.ftc.teamcode.Robot.touch;
import static org.firstinspires.ftc.teamcode.Robot.wrist;

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

@Autonomous(name = "FarAutoBlue", group = "Competition")
public class FarAutoBlue extends LinearOpMode {

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
        DELAY_START,
        LEFT_CLAW_OPEN,
        SLIDE_RETRACT,

    }

    private enum Board {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 180.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 96, 255);

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
    private static final int PIXEL_ARM_ANGLE = 3000;
    private static final int BACK_BOARD_ANGLE = 0;
    private static final int SLIDE_EXTENDED = 1800;
    private static final int SLIDE_START_POS = 0;
    private FarAutoBlue.State CurrentState;
    static boolean pressed = false;




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
                if (gamepad1.b) {
                    leftPark = !leftPark;
                }
            }
        }

        /* Trajectories either consist of vectors or poses. Vectors are for moving only x and y coordinates while poses have a heading(angle)
            For example
            a pose at coordinates (10,-10) facing 120 degrees would look like
            Pose2d myPose = new Pose2d(10,-10, Math.toRadians(120));
            Assuming you start at (0,0) at the start of the program, the robot with move to the coordinates labeled at an 120 degree heading
         */

        waitForStart();

        if (opModeIsActive()) {

            runtime.reset();
            StateTime.reset();
            delayTimer.reset();
            FarAutoBlue.Board board = FarAutoBlue.Board.LEFT;
            FarAutoBlue.State state = FarAutoBlue.State.INITIAL;


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


                Pose2d startPoseBlue = new Pose2d(-36.2, 61, Math.toRadians(270));

                drive.setPoseEstimate(startPoseBlue);

                // tapes...

                TrajectorySequence leftTape = drive.trajectorySequenceBuilder(startPoseBlue)

                        .lineToLinearHeading(new Pose2d(-39, 48, Math.toRadians(297)))
                        .build();

                TrajectorySequence middleTape = drive.trajectorySequenceBuilder(startPoseBlue)

                        .lineToConstantHeading(new Vector2d(-35, 44))
                        .build();

                TrajectorySequence rightTape = drive.trajectorySequenceBuilder(startPoseBlue)

                        .lineToLinearHeading(new Pose2d(-38, 53, Math.toRadians(245)))
                        .build();

                // white pixel stacks (R,L, & M indicating starting point, all same end location):

                TrajectorySequence whiteStackL = drive.trajectorySequenceBuilder(leftTape.end())
                        .lineToLinearHeading(new Pose2d(-52, 48.5, Math.toRadians(212)))
                        .build();

                TrajectorySequence whiteStackM = drive.trajectorySequenceBuilder(middleTape.end())
                        .lineToLinearHeading(new Pose2d(-52, 48.5, Math.toRadians(212)))
                        .build();

                TrajectorySequence whiteStackR = drive.trajectorySequenceBuilder(rightTape.end())
                        .lineToLinearHeading(new Pose2d(-52, 48.5, Math.toRadians(212)))
                        .build();

                // transition moves (to get to position before crossing mid):

                TrajectorySequence posToCrossNonR = drive.trajectorySequenceBuilder(new Pose2d(-52, 48.5, Math.toRadians(212)))
                        .lineToLinearHeading(new Pose2d(-52, 12, Math.toRadians(33)))
                        .turn(Math.toRadians(-33))
                        .build();

                // need seperate one for right placement to avoid prop...
                TrajectorySequence posToCrossR = drive.trajectorySequenceBuilder(new Pose2d(-52, 48.5, Math.toRadians(212)))
                        .lineToLinearHeading(new Pose2d(-34.5, 48.5, Math.toRadians(270)))
                        .lineToConstantHeading(new Vector2d(-34.5, 12)) // turning during this would hit prop sadge (according to MeepMeep)...
                        .turn(Math.toRadians(90))
                        .build();

                // backboard locations:

                TrajectorySequence backBoardLeft = drive.trajectorySequenceBuilder(leftTape.end())

                        .lineToLinearHeading(new Pose2d(45.5, 39.1, Math.toRadians(0)))
                        .build();

                TrajectorySequence backBoardMiddle = drive.trajectorySequenceBuilder(middleTape.end())
                        .lineToLinearHeading(new Pose2d(45.5, 31.8, Math.toRadians(0)))
                        .build();

                TrajectorySequence backBoardRight = drive.trajectorySequenceBuilder(rightTape.end())
                        .lineToLinearHeading(new Pose2d(45.5, 25.9, Math.toRadians(0)))
                        .build();



                // parking spots:

                TrajectorySequence BBLparkRight = drive.trajectorySequenceBuilder(backBoardLeft.end())

                        .lineToConstantHeading(new Vector2d(45.5, 5))
                        .build();

                TrajectorySequence BBMparkRight = drive.trajectorySequenceBuilder(backBoardMiddle.end())

                        .lineToConstantHeading(new Vector2d(45.5, 5))
                        .build();

                TrajectorySequence BBRparkRight = drive.trajectorySequenceBuilder(backBoardRight.end())

                        .lineToConstantHeading(new Vector2d(45.5, 5))
                        .build();

                TrajectorySequence BBLparkLeft = drive.trajectorySequenceBuilder(backBoardLeft.end())

                        .lineToConstantHeading(new Vector2d(45.5, 60))
                        .build();

                TrajectorySequence BBMparkLeft = drive.trajectorySequenceBuilder(backBoardMiddle.end())

                        .lineToConstantHeading(new Vector2d(45.5, 60))
                        .build();

                TrajectorySequence BBRparkLeft = drive.trajectorySequenceBuilder(backBoardRight.end())

                        .lineToConstantHeading(new Vector2d(45.5, 60))
                        .build();


                switch (state) {

                    case DELAY_START:

                        if (delayStart && delayTimer.seconds() >= selectedDelayTime) {
                            state = FarAutoBlue.State.INITIAL;
                        } else if (!delayStart) {
                            state = FarAutoBlue.State.INITIAL;
                        }
                        break;

                    case INITIAL:

                        wrist.setPosition(1);
                        rightClaw.setPosition(0.70);
                        leftClaw.setPosition(0.09);
                        state = FarAutoBlue.State.TAPE_CAMERA;
                        break;

                    case TAPE_CAMERA:

                        if (myPipeline.getRectArea() > 2000) {
                            if (myPipeline.getRectMidpointX() > 240) {

                                webcam.stopStreaming();
                                StateTime.reset();
                                state = FarAutoBlue.State.LEFT_TAPE;

                            } else if (myPipeline.getRectMidpointX() > 0) {

                                webcam.stopStreaming();
                                StateTime.reset();
                                state = FarAutoBlue.State.MIDDLE_TAPE;

                            }
                        } else {

                            webcam.stopStreaming();
                            StateTime.reset();
                            state = FarAutoBlue.State.RIGHT_TAPE;

                        }

                        break;

                    case LEFT_TAPE:
                        drive.followTrajectorySequenceAsync(leftTape);
                        Subsystems.slideAngle(PIXEL_ARM_ANGLE);

                        if (!drive.isBusy()) {

                            StateTime.reset();
                            state = FarAutoBlue.State.SLIDE_EXTENSION;
                            board = FarAutoBlue.Board.LEFT;

                        }

                        break;

                    case MIDDLE_TAPE:
                        drive.followTrajectorySequenceAsync(middleTape);
                        Subsystems.slideAngle(PIXEL_ARM_ANGLE);

                        if (!drive.isBusy()) {

                            StateTime.reset();
                            state = FarAutoBlue.State.SLIDE_EXTENSION;
                            board = FarAutoBlue.Board.MIDDLE;

                        }

                        break;

                    case RIGHT_TAPE:
                        drive.followTrajectorySequenceAsync(rightTape);
                        Subsystems.slideAngle(PIXEL_ARM_ANGLE);

                        if (!drive.isBusy()) {

                            StateTime.reset();
                            state = FarAutoBlue.State.SLIDE_EXTENSION;
                            board = FarAutoBlue.Board.RIGHT;

                        }

                        break;

                    case SLIDE_EXTENSION:


                        if (StateTime.time() > 1) {
                            Subsystems.slideExtension(SLIDE_EXTENDED);
                        }
                        if (Math.abs(slideExtension.getCurrentPosition() - SLIDE_EXTENDED) <= 15) {

                            StateTime.reset();
                            state = FarAutoBlue.State.RIGHT_CLAW_OPEN;

                        }

                        break;

                    case RIGHT_CLAW_OPEN:

                        rightClaw.setPosition(0.3);

                        if (StateTime.time() > 0.5) {

                            StateTime.reset();
                            state = FarAutoBlue.State.BACKBOARD;

                        }

                        break;

                    case BACKBOARD:

                        Subsystems.slideAngle(BACK_BOARD_ANGLE);

                        switch (board) {
                            case RIGHT:
                                drive.followTrajectorySequenceAsync(backBoardRight);
                                break;
                            case MIDDLE:
                                drive.followTrajectorySequenceAsync(backBoardLeft);
                                break;
                            case LEFT:
                                drive.followTrajectorySequenceAsync(backBoardMiddle);
                                break;

                        }


                        if (!drive.isBusy()) {

                            StateTime.reset();
                            state = FarAutoBlue.State.LEFT_CLAW_OPEN;

                        }

                        break;

                    case LEFT_CLAW_OPEN:

                        leftClaw.setPosition(0.6);

                        if (StateTime.time() > 0.5) {

                            StateTime.reset();
                            state = FarAutoBlue.State.SLIDE_RETRACT;

                        }

                        break;

                    case SLIDE_RETRACT:

                        Subsystems.slideExtension(SLIDE_START_POS);

                        if (Math.abs(slideExtension.getCurrentPosition() - SLIDE_START_POS) <= 20) {

                            if (leftPark) {
                                if (board == FarAutoBlue.Board.LEFT) {
                                    drive.followTrajectorySequenceAsync(BBLparkLeft);
                                } else if (board == FarAutoBlue.Board.MIDDLE) {
                                    drive.followTrajectorySequenceAsync(BBMparkLeft);
                                } else {
                                    drive.followTrajectorySequenceAsync(BBRparkLeft);
                                }
                            } else {
                                if (board == FarAutoBlue.Board.LEFT) {
                                    drive.followTrajectorySequenceAsync(BBLparkRight);
                                } else if (board == FarAutoBlue.Board.MIDDLE) {
                                    drive.followTrajectorySequenceAsync(BBMparkRight);
                                } else {
                                    drive.followTrajectorySequenceAsync(BBRparkRight);
                                }

                                state = FarAutoBlue.State.PARK;

                            }

                        }

                        break;

                    case PARK:

                        drive.update();
                        break;

                    default:

                        StateTime.reset();
                        runtime.reset();

                }


            }
        }
    }
}