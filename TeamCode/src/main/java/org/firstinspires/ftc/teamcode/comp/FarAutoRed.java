package org.firstinspires.ftc.teamcode.comp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.Robot.*;



import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Vision.ContourPipelineRed;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.Subsystems;

@Autonomous(name = "FarAutoRed", group = "Competition")
public class FarAutoRed extends LinearOpMode {
    
    private enum State {
        INITIAL,
        BACKBOARD,
        PARK,
        DRIVE_THROUGH_CENTER,
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
    public enum Board {
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

    private OpenCvCamera webcam;
    public static PIDController controller;
    public static double p = 0.003, i = 0, d = 0.00015;
    public static double f = 0.08;
    public final double ticks_per_rev = 537.6;
    private static final int PIXEL_ARM_ANGLE = 3125;
    private static final int BACK_BOARD_ANGLE = 2400;
    private static final int ARM_RESTING_POSITION = 0;
    private static final int ARM_UNDER_BAR = 2200;
    private static final int SLIDE_EXTENDED = 1900;
    private static final int SLIDE_START_POS = 0;
    private static final int SLIDE_BACKBOARD = 1050;
    private static final int SLIDE_LEFT_TAPE = 1200;
    private static final int PIXEL_STACK_ANGLE = 3000;
    private static final int PIXEL_STACK_EXTENSION = 1300;
    private static final double LEFT_CLAW_OPEN = 0.6;
    private static final double LEFT_CLAW_CLOSE = 0.17;
    private static final double RIGHT_CLAW_OPEN = 0.3;
    private static final double RIGHT_CLAW_CLOSE = 0.6;
    private static final double WRIST_PIXEL_PICKUP = 0.46;
    private static final double WRIST_BACKBOARD = 0.73;
    private static final double WRIST_DOWN = 0;
    private static final double WRIST_UP = 1;
    private static final double WRIST_LEFT_TAPE = 0.5;

    static boolean pressed = false;
    protected SampleMecanumDrive drive;
    Pose2d startPoseBlueBB = new Pose2d(16, 61, Math.toRadians(270));




    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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
        rightClaw.setPosition(RIGHT_CLAW_CLOSE);
        leftClaw.setPosition(LEFT_CLAW_CLOSE);
        // Touch Sensor
        touch = hardwareMap.get(TouchSensor.class, "touch");

        Subsystems.init();

        drive.setPoseEstimate(startPoseBlueBB);
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipelineRed myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipelineRed(borderLeftX, borderRightX, borderTopY, borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
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
        telemetry.update();
        
        
        if (gamepad1.a) {
            delayStart = !delayStart;
        }

        // Allow the user to select the delay time using the controller
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

        telemetry.addData("Delay Start", delayStart ? "Yes" : "No");
        telemetry.addData("Selected Delay Time", "%.1f seconds", selectedDelayTime);
        telemetry.addData("Press B to toggle park side", "Current: " + (leftPark ? "Left" : "Right"));
        telemetry.update();
        

        waitForStart();
        if (opModeIsActive()) {

            runtime.reset();
            StateTime.reset();
            delayTimer.reset();
            Board board = Board.LEFT;
            State state;
            if (delayStart) {
                state = FarAutoRed.State.DELAY_START;
            } else {
                state = FarAutoRed.State.INITIAL;
            }


            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("CurrentState", state.toString());
                telemetry.addData("Status", "Run Time: " + runtime);
                telemetry.addData("posleftslide", slideExtension.getCurrentPosition());
                telemetry.addData("posarmmotor", slidePivot.getCurrentPosition());
                telemetry.update();


                Pose2d startPoseRed = new Pose2d(-36.2, -61, Math.toRadians(90));

                drive.setPoseEstimate(startPoseRed);


                TrajectorySequence leftTape = drive.trajectorySequenceBuilder(startPoseRed)

                        .lineToLinearHeading(new Pose2d(-38, 46, Math.toRadians(310)))
                        .build();

                TrajectorySequence leftTape2 = drive.trajectorySequenceBuilder(startPoseRed)

                        .lineToLinearHeading(new Pose2d(-35,-35, Math.toRadians(180)))
                        .build();

                TrajectorySequence middleTape = drive.trajectorySequenceBuilder(startPoseRed)

                        .lineToLinearHeading(new Pose2d(-47.2, -57, Math.toRadians(75)))
                        .build();

                TrajectorySequence rightTape = drive.trajectorySequenceBuilder(startPoseRed)

                        .lineToLinearHeading(new Pose2d(-54.2, -43, Math.toRadians(23)))
                        .build();

                TrajectorySequence backBoardLeft = drive.trajectorySequenceBuilder(rightTape.end())

                        .lineToLinearHeading(new Pose2d(45.5, -25.9, Math.toRadians(0)))
                        .build();

                TrajectorySequence backBoardMiddle = drive.trajectorySequenceBuilder(middleTape.end())

                        .lineToLinearHeading(new Pose2d(45.5, -31.8, Math.toRadians(0)))
                        .build();

                TrajectorySequence backBoardRight = drive.trajectorySequenceBuilder(leftTape.end())

                        .lineToLinearHeading(new Pose2d(45.5, -39.1, Math.toRadians(0)))
                        .build();

                TrajectorySequence BBLparkRight = drive.trajectorySequenceBuilder(backBoardLeft.end())

                        .lineToConstantHeading(new Vector2d(45.5, -60))
                        .build();

                TrajectorySequence BBMparkRight = drive.trajectorySequenceBuilder(backBoardMiddle.end())

                        .lineToConstantHeading(new Vector2d(45.5, -60))
                        .build();

                TrajectorySequence BBRparkRight = drive.trajectorySequenceBuilder(backBoardRight.end())

                        .lineToConstantHeading(new Vector2d(45.5, -60))
                        .build();

                TrajectorySequence BBLparkLeft = drive.trajectorySequenceBuilder(backBoardLeft.end())

                        .lineToConstantHeading(new Vector2d(45.5, -5))
                        .build();

                TrajectorySequence BBMparkLeft = drive.trajectorySequenceBuilder(backBoardMiddle.end())

                        .lineToConstantHeading(new Vector2d(45.5, -5))
                        .build();

                TrajectorySequence BBRparkLeft = drive.trajectorySequenceBuilder(backBoardRight.end())

                        .lineToConstantHeading(new Vector2d(45.5, -5))
                        .build();
                TrajectorySequence Drive;



                switch (state) {

                    case DELAY_START:

                        if (delayStart && delayTimer.seconds() >= selectedDelayTime) {
                            state = State.INITIAL;
                        } else if (!delayStart) {
                            state = State.INITIAL;
                        }
                        break;

                    case INITIAL:

                        wrist.setPosition(0);
                        rightClaw.setPosition(0.70);
                        leftClaw.setPosition(0.09);
                        state = State.TAPE_CAMERA;
                        break;

                    case TAPE_CAMERA:

                        if (myPipeline.getRectArea() > 2000) {
                            if (myPipeline.getRectMidpointX() > 240) {

                                webcam.stopStreaming();
                                StateTime.reset();
                                state = State.LEFT_TAPE;

                            } else if (myPipeline.getRectMidpointX() > 0) {

                                webcam.stopStreaming();
                                StateTime.reset();
                                state = State.MIDDLE_TAPE;

                            }
                        } else {

                            webcam.stopStreaming();
                            StateTime.reset();
                            state = State.RIGHT_TAPE;

                        }

                        break;

                    case LEFT_TAPE:

                        drive.followTrajectorySequenceAsync(leftTape);
                        Subsystems.slideAngle(PIXEL_ARM_ANGLE);

                        if (!drive.isBusy() && Math.abs(slidePivot.getCurrentPosition() - PIXEL_ARM_ANGLE) <= 20) {

                            StateTime.reset();
                            state= State.SLIDE_EXTENSION;
                            board = Board.LEFT;

                        }

                        break;

                    case MIDDLE_TAPE:

                        drive.followTrajectorySequenceAsync(middleTape);
                        Subsystems.slideAngle(PIXEL_ARM_ANGLE);

                        if (!drive.isBusy() && Math.abs(slideExtension.getCurrentPosition() - PIXEL_ARM_ANGLE) <= 20) {

                            StateTime.reset();
                            state= State.SLIDE_EXTENSION;
                            board = Board.MIDDLE;

                        }

                        break;

                    case RIGHT_TAPE:

                        drive.followTrajectorySequenceAsync(rightTape);
                        Subsystems.slideAngle(PIXEL_ARM_ANGLE);

                        if (!drive.isBusy() && Math.abs(slideExtension.getCurrentPosition() - PIXEL_ARM_ANGLE) <= 20) {

                            StateTime.reset();
                            state= State.SLIDE_EXTENSION;
                            board = Board.RIGHT;

                        }

                        break;

                    case SLIDE_EXTENSION:

                        Subsystems.slideExtension(SLIDE_EXTENDED);

                        if (Math.abs(slideExtension.getCurrentPosition() - SLIDE_EXTENDED) <= 15) {

                            StateTime.reset();
                            state = State.RIGHT_CLAW_OPEN;

                        }

                        break;

                    case RIGHT_CLAW_OPEN:

                        rightClaw.setPosition(0.2);

                        if (StateTime.time() > 0.5) {

                            StateTime.reset();
                            state = State.BACKBOARD;

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
                            state = State.LEFT_CLAW_OPEN;

                        }

                        break;

                    case LEFT_CLAW_OPEN:

                        leftClaw.setPosition(0.5);

                        if (StateTime.time() > 0.5) {

                            StateTime.reset();
                            state = State.SLIDE_RETRACT;

                        }

                        break;

                    case SLIDE_RETRACT:

                        Subsystems.slideExtension(SLIDE_START_POS);

                        if (Math.abs(slidePivot.getCurrentPosition() - SLIDE_START_POS) <= 20) {

                            if (leftPark) {
                                if (board == Board.LEFT) {
                                    drive.followTrajectorySequenceAsync(BBLparkLeft);
                                } else if (board == Board.MIDDLE) {
                                    drive.followTrajectorySequenceAsync(BBMparkLeft);
                                } else if (board == Board.RIGHT) {
                                    drive.followTrajectorySequenceAsync(BBRparkLeft);
                                }
                            } else {
                                if (board == Board.LEFT) {
                                    drive.followTrajectorySequenceAsync(BBLparkRight);
                                } else if (board == Board.MIDDLE) {
                                    drive.followTrajectorySequenceAsync(BBMparkRight);
                                } else if (board == Board.RIGHT) {
                                    drive.followTrajectorySequenceAsync(BBRparkRight);
                                }

                                state = State.PARK;

                                break;

                            }

                        }

                        break;

                    case PARK:

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