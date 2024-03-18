    package org.firstinspires.ftc.teamcode.comp;

    import static org.firstinspires.ftc.teamcode.PoseStorage.currentPose;
    import static org.firstinspires.ftc.teamcode.Robot.leftClaw;
    import static org.firstinspires.ftc.teamcode.Robot.leftFront;
    import static org.firstinspires.ftc.teamcode.Robot.leftRear;
    import static org.firstinspires.ftc.teamcode.Robot.rightClaw;
    import static org.firstinspires.ftc.teamcode.Robot.rightFront;
    import static org.firstinspires.ftc.teamcode.Robot.rightRear;
    import static org.firstinspires.ftc.teamcode.Robot.slideExtension;
    import static org.firstinspires.ftc.teamcode.Robot.slidePivot;
    import static org.firstinspires.ftc.teamcode.Robot.touch;
    import static org.firstinspires.ftc.teamcode.Robot.wrist;

    import com.acmerobotics.dashboard.FtcDashboard;
    import com.acmerobotics.dashboard.config.Config;
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
    import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive2;
    import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
    import org.opencv.core.Scalar;
    import org.openftc.easyopencv.OpenCvCamera;
    import org.openftc.easyopencv.OpenCvCameraFactory;
    import org.openftc.easyopencv.OpenCvCameraRotation;

    @Config
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
            TRANSITION_POS,
            DELAY_START,
            LEFT_CLAW_OPEN,
            SLIDE_RETRACT, GOING_TO_BACKBOARD, DONE, RIGHT_TAPE_ADJUSTMENT,

        }

        private enum Board {
            LEFT,
            MIDDLE,
            RIGHT
        }
/*
        public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 180.0);
        public static Scalar scalarUpperYCrCb = new Scalar(255.0, 96, 255);
*/
        public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0, 140);
        public static Scalar scalarUpperYCrCb = new Scalar(255.0, 80, 255);
        public final ElapsedTime runtime = new ElapsedTime();
        private final ElapsedTime StateTime = new ElapsedTime();
        ElapsedTime delayTimer = new ElapsedTime();
        boolean delayStart = false; // Default: no delay
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
        private static final int SLIDE_EXTENDED = 1800;
        private static final int SLIDE_START_POS = 10;
        private static final int SLIDE_BACKBOARD = 925;
        private static final int SLIDE_RIGHT_TAPE = 1200;
        private static final int PIXEL_STACK_ANGLE = 3000;
        private static final int PIXEL_STACK_EXTENSION = 1300;
        private static final double LEFT_CLAW_OPEN = 0.54;
        private static final double LEFT_CLAW_CLOSE = 0.17;
        private static final double RIGHT_CLAW_OPEN = 0.3;
        private static final double RIGHT_CLAW_CLOSE = 0.6;
        private static final double WRIST_PIXEL_PICKUP = 0.46;
        private static final double WRIST_BACKBOARD = 0.71;
        private static final double WRIST_DOWN = 0;
        private static final double WRIST_UP = 1;
        private static final double WRIST_RIGHT_TAPE = 0.5;
        static boolean pressed = false;
        protected SampleMecanumDrive2 drive;
        Pose2d startPoseBlue = new Pose2d(-36.2, 61, Math.toRadians(270));


        @Override
        public void runOpMode() throws InterruptedException {

            drive = new SampleMecanumDrive2(hardwareMap);

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

            drive.setPoseEstimate(startPoseBlue);

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
                telemetry.addData("Press B to toggle park side", "Current: " + (leftPark ? "Right" : "Left"));
                telemetry.update();
                // Allow the user to select the delay time using the controller
                if (gamepad1.a) {
                    delayStart = !delayStart;
                }
                if (delayStart) {
                    if (gamepad1.right_bumper) {
                        selectedDelayTime += 1.0;
                        sleep(200); // Add a small delay to prevent rapid decrements
                    } else if (gamepad1.left_bumper && selectedDelayTime > 1.0) {
                        selectedDelayTime -= 1.0;
                        sleep(200); // Add a small delay to prevent rapid decrements
                    }
                }
                if (gamepad1.b) {
                    leftPark = !leftPark;
                }
            }

            TrajectorySequence leftTape = drive.trajectorySequenceBuilder(startPoseBlue)

                    .lineToLinearHeading(new Pose2d(-47.6, 48, Math.toRadians(320)))
                    .build();

            TrajectorySequence middleTape = drive.trajectorySequenceBuilder(startPoseBlue)

                    .lineToConstantHeading(new Vector2d(-35, 53))
                    .build();

            TrajectorySequence rightTape = drive.trajectorySequenceBuilder(startPoseBlue)

                    .lineToConstantHeading(new Vector2d(-45.2, 59))
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


                    TrajectorySequence parkRight = drive.trajectorySequenceBuilder(currentPose)
                            .lineToLinearHeading(new Pose2d(51, 8, Math.toRadians(180)))
                            .build();

                    TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(currentPose)
                            .lineToLinearHeading(new Pose2d(51, 55, Math.toRadians(180)))
                            .build();

                    TrajectorySequence whiteStack = drive.trajectorySequenceBuilder(currentPose)
                            .lineToLinearHeading(new Pose2d(-51.5,32, Math.toRadians(180)))
                            .build();

                    // transition moves (to get to position before crossing mid):


                    TrajectorySequence poseToCross = drive.trajectorySequenceBuilder(currentPose)
                            //.lineToLinearHeading(new Pose2d(-52, 12, Math.toRadians(33)))
                            .lineToLinearHeading(new Pose2d(-57, 7, Math.toRadians(0)))
                            .build();
                    TrajectorySequence righttapefix = drive.trajectorySequenceBuilder(rightTape.end())
                            .strafeLeft(10)
                            .lineToLinearHeading(new Pose2d(-35, 7, Math.toRadians(270)))
                            .turn(Math.toRadians(90))
                            .build();

                    TrajectorySequence poseToCrossTight = drive.trajectorySequenceBuilder(currentPose)
                            .turn(Math.toRadians(20))
                            .lineToLinearHeading(new Pose2d(-35, 7, Math.toRadians(270)))
                            .build();

                    TrajectorySequence crossMap = drive.trajectorySequenceBuilder(currentPose)
                            .lineToLinearHeading(new Pose2d(40, 7, Math.toRadians(0)))
                            .build();

                    TrajectorySequence backBoardLeft = drive.trajectorySequenceBuilder(crossMap.end())
                            //.lineToLinearHeading(new Pose2d(48, 40, Math.toRadians(0)))
                            .lineToLinearHeading(new Pose2d(49.4, 38.1, Math.toRadians(0)))
                            .build();

                    TrajectorySequence backBoardMiddle = drive.trajectorySequenceBuilder(crossMap.end())
                            .lineToLinearHeading(new Pose2d(49.4, 31.8, Math.toRadians(0)))
                            .build();

                    TrajectorySequence backBoardRight = drive.trajectorySequenceBuilder(crossMap.end())
                            .lineToLinearHeading(new Pose2d(48, 28, Math.toRadians(0)))
                            .build();



                    switch (state) {

                        case DELAY_START:

                            if (delayStart && delayTimer.seconds() >= selectedDelayTime) {
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

                                    drive.followTrajectorySequenceAsync(leftTape);
                                    webcam.stopStreaming();
                                    StateTime.reset();
                                    state = State.LEFT_TAPE;

                                } else if (myPipeline.getRectMidpointX() > 275) {

                                    drive.followTrajectorySequenceAsync(middleTape);
                                    webcam.stopStreaming();
                                    StateTime.reset();
                                    state = State.MIDDLE_TAPE;

                                }
                            } else {

                                drive.followTrajectorySequenceAsync(rightTape);
                                webcam.stopStreaming();
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
                            wrist.setPosition(WRIST_RIGHT_TAPE);

                            if (Math.abs(slidePivot.getCurrentPosition()) >= 2990) {
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

                            rightClaw.setPosition(RIGHT_CLAW_OPEN);

                            if (StateTime.time() > 2) {

                                Subsystems.slideExtension(SLIDE_START_POS);

                                if (board == Board.RIGHT) {

                                    drive.followTrajectorySequenceAsync(righttapefix);
                                    StateTime.reset();
                                    state = State.GOING_TO_BACKBOARD;

                                } else {

                                        drive.followTrajectorySequenceAsync(poseToCross);
                                        StateTime.reset();
                                        state = State.GOING_TO_BACKBOARD;

                                }
                            }

                            break;


                        case GOING_TO_BACKBOARD:

                            Subsystems.slideAngle(ARM_UNDER_BAR);
                            wrist.setPosition(WRIST_DOWN);


                            if (!drive.isBusy()) {

                                drive.followTrajectorySequenceAsync(crossMap);
                                StateTime.reset();
                                state = State.BACKBOARD;

                            }

                            break;

                        case BACKBOARD:

                            if (!drive.isBusy()) {

                                if (board == Board.LEFT) {

                                    drive.followTrajectorySequence(backBoardLeft);

                                }
                                if (board == Board.MIDDLE) {

                                    drive.followTrajectorySequence(backBoardMiddle);

                                }
                                if (board == Board.RIGHT) {

                                    drive.followTrajectorySequence(backBoardRight);

                                }

                                if (!drive.isBusy()) {

                                    Subsystems.slideAngle(BACK_BOARD_ANGLE);
                                    wrist.setPosition(WRIST_BACKBOARD);
                                    StateTime.reset();
                                    state = State.LEFT_CLAW_OPEN;

                                }
                            }

                            break;

                        case LEFT_CLAW_OPEN:

                            if (StateTime.time() > 1) {
                                Subsystems.slideExtension(SLIDE_BACKBOARD);
                            }

                            if (StateTime.time() > 2 ) {

                                leftClaw.setPosition(LEFT_CLAW_OPEN);
                                StateTime.reset();
                                state = State.SLIDE_RETRACT;

                            }

                            break;

                        case SLIDE_RETRACT:

                            if (StateTime.time() > 0.5) {
                                Subsystems.slideExtension(SLIDE_START_POS);
                            }
                            if (Math.abs(slideExtension.getCurrentPosition()) <= 20) {

                                Subsystems.slideAngle(ARM_RESTING_POSITION);
                                wrist.setPosition(WRIST_UP);
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