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
import static org.firstinspires.ftc.teamcode.RobotHardware.*;


import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.ContourPipelineBlue;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.Subsystems;

@Autonomous(name = "AutoRedBoard", group = "Competition")
public class AutoRedBoard extends LinearOpMode {

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
    private static final int PIXEL_ARM_ANGLE = 1700;
    private static final int BACK_BOARD_ANGLE = 0;
    private static final int SLIDE_EXTENDED = 4400;
    private static final int SLIDE_START_POS = 50;
    private State CurrentState;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        initializeHardware();

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
        FtcDashboard.getInstance().startCameraStream(webcam, 30);
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
            State state = State.DELAY_START;
            Board board = Board.LEFT;


            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("CurrentState", CurrentState.toString());
                telemetry.addData("Status", "Run Time: " + runtime);
                telemetry.addData("posleftslide", leftSlide.getCurrentPosition());
                telemetry.addData("posrightslide", rightSlide.getCurrentPosition());
                telemetry.addData("posarmmotor", armmotor.getCurrentPosition());
                telemetry.update();


                Pose2d startPoseRedBB = new Pose2d(16, -61, Math.toRadians(90));

                drive.setPoseEstimate(startPoseRedBB);


                TrajectorySequence leftTape = drive.trajectorySequenceBuilder(startPoseRedBB)

                        .lineToLinearHeading(new Pose2d(34, -43, Math.toRadians(157)))

                        .build();

                TrajectorySequence middleTape = drive.trajectorySequenceBuilder(startPoseRedBB)

                        .lineToLinearHeading(new Pose2d(27, 57, Math.toRadians(255)))
                        .build();

                TrajectorySequence rightTape = drive.trajectorySequenceBuilder(startPoseRedBB)

                        .lineToConstantHeading(new Vector2d(26.3, -55))
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



                switch (CurrentState) {

                    case DELAY_START:

                        if (delayStart && delayTimer.seconds() >= selectedDelayTime) {
                            state = State.INITIAL;
                        } else if (!delayStart) {
                            state = State.INITIAL;
                        }
                        break;

                    case INITIAL:

                        Subsystems.syncedWrist(0);
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
                        Subsystems.armPosition(PIXEL_ARM_ANGLE);

                        if (!drive.isBusy() && Math.abs(armmotor.getCurrentPosition() - PIXEL_ARM_ANGLE) <= 20) {

                            StateTime.reset();
                            state= State.SLIDE_EXTENSION;
                            board = Board.LEFT;

                        }

                        break;

                    case MIDDLE_TAPE:

                        drive.followTrajectorySequenceAsync(middleTape);
                        Subsystems.armPosition(PIXEL_ARM_ANGLE);

                        if (!drive.isBusy() && Math.abs(armmotor.getCurrentPosition() - PIXEL_ARM_ANGLE) <= 20) {

                            StateTime.reset();
                            state= State.SLIDE_EXTENSION;
                            board = Board.MIDDLE;

                        }

                        break;

                    case RIGHT_TAPE:

                        drive.followTrajectorySequenceAsync(rightTape);
                        Subsystems.armPosition(PIXEL_ARM_ANGLE);

                        if (!drive.isBusy() && Math.abs(armmotor.getCurrentPosition() - PIXEL_ARM_ANGLE) <= 20) {

                            StateTime.reset();
                            state= State.SLIDE_EXTENSION;
                            board = Board.RIGHT;

                        }

                        break;

                    case SLIDE_EXTENSION:

                        Subsystems.slideRightPosition(SLIDE_EXTENDED);

                        if (Math.abs(rightSlide.getCurrentPosition() - SLIDE_EXTENDED) <= 15) {

                            rightClaw.setPosition(0.2);
                            StateTime.reset();
                            state = State.RIGHT_CLAW_OPEN;

                        }

                        break;

                    case RIGHT_CLAW_OPEN:

                        if (StateTime.time() > 0.5) {

                            StateTime.reset();
                            state = State.BACKBOARD;

                        }

                        break;

                    case BACKBOARD:

                        switch (board) {
                            case RIGHT:
                                drive.followTrajectorySequenceAsync(backBoardRight);
                                break;
                            case MIDDLE:
                                drive.followTrajectorySequenceAsync(backBoardLeft);
                                break;
                            case LEFT:
                                drive.followTrajectorySequenceAsync(backBoardLeft);
                                break;

                        }

                        Subsystems.armPosition(BACK_BOARD_ANGLE);

                        if (!drive.isBusy()) {

                            leftClaw.setPosition(0.5);
                            StateTime.reset();
                            state = State.LEFT_CLAW_OPEN;

                        }

                        break;

                    case LEFT_CLAW_OPEN:

                        if (StateTime.time() > 0.5) {

                            Subsystems.slideRightPosition(SLIDE_START_POS);
                            StateTime.reset();
                            state = State.SLIDE_RETRACT;

                        }

                        break;

                    case SLIDE_RETRACT:
                        if (Math.abs(rightSlide.getCurrentPosition() - SLIDE_START_POS) <= 20) {

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

                        state = State.INITIAL;
                        StateTime.reset();
                        runtime.reset();

                }

                drive.update();


            }
        }
    }

}

