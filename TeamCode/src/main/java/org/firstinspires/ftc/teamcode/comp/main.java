    package org.firstinspires.ftc.teamcode.comp;


    import com.acmerobotics.roadrunner.geometry.Pose2d;
    import com.arcrobotics.ftclib.controller.PIDController;
    import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.IMU;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.hardware.TouchSensor;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
    import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

    import java.lang.Math;

    import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
    import org.firstinspires.ftc.teamcode.PoseStorage;
    import org.firstinspires.ftc.teamcode.Subsystems;
    import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

    import static org.firstinspires.ftc.teamcode.Robot.*;


    @TeleOp(name = "main", group = "Competition")
    public class main extends LinearOpMode {

        private enum State {
            INITIAL,
            PIXEL_PICKUP,
            BACKBOARD,
            HANG,
            DRONE,
            UNDER_BAR,
            LOWER_BACKBOARD
        }
        public final ElapsedTime runtime = new ElapsedTime();
        private final ElapsedTime StateTime = new ElapsedTime();
        public static PIDController controller;
        public static double p = 1, i = 0, d = 0.00015;
        public static double f = 0.08;
        public final double ticks_per_rev = 537.6;
        static boolean pressed = false;
        static boolean leftClawPressed = false;
        static boolean rightClawPressed = false;
        private static final int PIXEL_ARM_ANGLE = 3125;
        private static final int BACK_BOARD_ANGLE = 1200;
        private static final int ARM_START_POSITION = 0;
        private static final int ARM_LOWER_BACKBOARD = 2200;
        private static final int ARM_RESTING = 2200;
        private static final int SLIDE_EXTENDED = 1800;
        private static final int SLIDE_BACKBOARD = 1600;

        private static final int SLIDE_START_POS = 10;
        private static final double LEFT_CLAW_OPEN = 0.54;
        private static final double LEFT_CLAW_CLOSE = 0.17;
        private static final double RIGHT_CLAW_OPEN = 0.2;
        private static final double RIGHT_CLAW_CLOSE = 0.6;
        private static final double WRIST_PIXEL_PICKUP = 0.43;
        private static final double WRIST_BACKBOARD = 0.55;
        private static final double WRIST_DOWN = 0;
        private static final double WRIST_UP = 1;


        protected SampleMecanumDrive drive;


        @Override
        public void runOpMode() throws InterruptedException {

            drive = new SampleMecanumDrive(hardwareMap);

            // Drive Motors
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
            rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            // Slide Motors
            slideExtension = hardwareMap.get(DcMotorEx.class, "slideExtension");
            slideExtension.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            slideExtension.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            hangingMotor = hardwareMap.get(DcMotorEx.class, "hangingMotor");
            hangingMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            hangingMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slidePivot = hardwareMap.get(DcMotorEx.class, "slidePivot");
            slidePivot.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            slidePivot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            // Servos
            wrist = hardwareMap.get(Servo.class, "wrist");
            leftClaw = hardwareMap.get(Servo.class, "leftClaw");
            rightClaw = hardwareMap.get(Servo.class, "rightClaw");
            drone = hardwareMap.get(Servo.class, "drone");
            drone.setPosition(0.2);
            rightClaw.setPosition(0.6);
            leftClaw.setPosition(0.17);

            //Sensor
            touch = hardwareMap.get(TouchSensor.class, "touch");

            IMU imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
            imu.initialize(parameters);
            controller = new PIDController(p, i, d);
            Subsystems.init();




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




            waitForStart();
            if (opModeIsActive()) {

                runtime.reset();
                State state = State.INITIAL;
                StateTime.reset();

                while (opModeIsActive()) {
                    telemetry.addData("Status", "State Time: " + StateTime);
                    telemetry.addData("Status", "State: " + state);
                    telemetry.addData("posslide", slideExtension.getCurrentPosition());
                    telemetry.addData("poshangingMotor", hangingMotor.getCurrentPosition());
                    telemetry.addData("posslidePivot", slidePivot.getCurrentPosition());
                    telemetry.addData("posleftWrist", wrist.getPosition());
                    telemetry.addData("slidePower", slideExtension.getPower());
                    telemetry.addData("Touch Sensor State", touch.isPressed());
                    telemetry.update();

                    if (touch.isPressed() && !pressed) {
                        slideExtension.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        slideExtension.setPower(0);
                        pressed = true;
                    } else if (!touch.isPressed()) {
                        // Reset 'pressed' when the touch sensor is released
                        pressed = false;
                    }
                    if (gamepad1.right_trigger > 0.5 && gamepad1.left_trigger > 0.5) {

                        StateTime.reset();
                        state = State.INITIAL;

                    }

                    switch (state) {

                        case INITIAL:

                            Subsystems.hangingArm(0);
                            Subsystems.slideAngle(0);
                            Subsystems.slideExtension(0);

                            if (slideExtension.getCurrentPosition() <= 60) {
                                wrist.setPosition(WRIST_DOWN);
                            }

                            if (gamepad1.right_bumper && Math.abs(slidePivot.getCurrentPosition() - ARM_START_POSITION) <= 10 && Math.abs(slideExtension.getCurrentPosition() - SLIDE_START_POS) <= 60) {

                                StateTime.reset();
                                state = State.BACKBOARD;

                            }
                            if (gamepad1.b && Math.abs(slidePivot.getCurrentPosition() - ARM_START_POSITION) <= 10 && Math.abs(slideExtension.getCurrentPosition() - SLIDE_START_POS) <= 60) {

                                StateTime.reset();
                                state = State.PIXEL_PICKUP;

                            }
                            if (gamepad2.y && Math.abs(slidePivot.getCurrentPosition() - ARM_START_POSITION) <= 10 && Math.abs(slideExtension.getCurrentPosition() - SLIDE_START_POS) <= 60) {

                                StateTime.reset();
                                state = State.DRONE;

                            }
                            if (gamepad1.dpad_left) {

                                StateTime.reset();
                                state = State.UNDER_BAR;

                            }
                            break;

                        case BACKBOARD:

                            Subsystems.slideAngle(BACK_BOARD_ANGLE);
                            Subsystems.slideExtension(SLIDE_BACKBOARD);
                            wrist.setPosition(WRIST_BACKBOARD);


                            if (gamepad1.left_bumper) {

                                StateTime.reset();
                                state = State.UNDER_BAR;

                            }

                            if (gamepad1.dpad_down) {

                               StateTime.reset();
                               state = State.LOWER_BACKBOARD;

                            }
                            if (gamepad2.a) {

                                rightClaw.setPosition(RIGHT_CLAW_OPEN);
                                leftClaw.setPosition(LEFT_CLAW_OPEN);

                            }
                            if (gamepad2.b) {

                                rightClaw.setPosition(RIGHT_CLAW_CLOSE);
                                leftClaw.setPosition(LEFT_CLAW_CLOSE);

                            }
                            if (gamepad1.left_trigger > 0.5) {

                                StateTime.reset();
                                state = State.LOWER_BACKBOARD;

                            }

                            if (gamepad2.right_bumper) {
                                if (!rightClawPressed) {
                                    rightClawPressed = true; // Set the flag to true when the bumper is initially pressed
                                    // Toggle the right claw position when the bumper is pressed
                                    if (rightClaw.getPosition() == RIGHT_CLAW_OPEN) {
                                        rightClaw.setPosition(RIGHT_CLAW_CLOSE);
                                    } else {
                                        rightClaw.setPosition(RIGHT_CLAW_OPEN);
                                    }
                                }
                            } else {
                                rightClawPressed = false; // Reset the flag when the bumper is released
                            }

                            // Check the left bumper on gamepad2 to control the left claw
                            if (gamepad2.left_bumper) {
                                if (!leftClawPressed) {
                                    leftClawPressed = true; // Set the flag to true when the bumper is initially pressed
                                    // Toggle the left claw position when the bumper is pressed
                                    if (leftClaw.getPosition() == LEFT_CLAW_OPEN) {
                                        leftClaw.setPosition(LEFT_CLAW_CLOSE);
                                    } else {
                                        leftClaw.setPosition(LEFT_CLAW_OPEN);
                                    }
                                }
                            } else {
                                leftClawPressed = false; // Reset the flag when the bumper is released
                            }

                            break;

                        case LOWER_BACKBOARD:

                            Subsystems.slideAngle(ARM_LOWER_BACKBOARD);
                            Subsystems.slideExtension(SLIDE_BACKBOARD);
                            wrist.setPosition(WRIST_BACKBOARD);


                            if (gamepad1.left_bumper) {

                                StateTime.reset();
                                state = State.UNDER_BAR;

                            }
                            if (gamepad1.dpad_up) {

                                StateTime.reset();
                                state = State.BACKBOARD;

                            }
                            if (gamepad2.a) {

                                rightClaw.setPosition(RIGHT_CLAW_OPEN);
                                leftClaw.setPosition(LEFT_CLAW_OPEN);

                            }
                            if (gamepad2.b) {

                                rightClaw.setPosition(RIGHT_CLAW_CLOSE);
                                leftClaw.setPosition(LEFT_CLAW_CLOSE);

                            }

                            if (gamepad2.right_bumper) {
                                if (!rightClawPressed) {
                                    rightClawPressed = true; // Set the flag to true when the bumper is initially pressed
                                    // Toggle the right claw position when the bumper is pressed
                                    if (rightClaw.getPosition() == RIGHT_CLAW_OPEN) {
                                        rightClaw.setPosition(RIGHT_CLAW_CLOSE);
                                    } else {
                                        rightClaw.setPosition(RIGHT_CLAW_OPEN);
                                    }
                                }
                            } else {
                                rightClawPressed = false; // Reset the flag when the bumper is released
                            }

                            // Check the left bumper on gamepad2 to control the left claw
                            if (gamepad2.left_bumper) {
                                if (!leftClawPressed) {
                                    leftClawPressed = true; // Set the flag to true when the bumper is initially pressed
                                    // Toggle the left claw position when the bumper is pressed
                                    if (leftClaw.getPosition() == LEFT_CLAW_OPEN) {
                                        leftClaw.setPosition(LEFT_CLAW_CLOSE);
                                    } else {
                                        leftClaw.setPosition(LEFT_CLAW_OPEN);
                                    }
                                }
                            } else {
                                leftClawPressed = false; // Reset the flag when the bumper is released
                            }

                            break;


                        case PIXEL_PICKUP:

                            Subsystems.slideAngle(PIXEL_ARM_ANGLE);
                            wrist.setPosition(WRIST_PIXEL_PICKUP);
                            Subsystems.slideExtension(SLIDE_EXTENDED);


                            if (gamepad1.x) {

                                StateTime.reset();
                                state = State.INITIAL;

                            }
                            if (gamepad2.a) {

                                rightClaw.setPosition(RIGHT_CLAW_OPEN);
                                leftClaw.setPosition(LEFT_CLAW_OPEN);

                            }
                            if (gamepad2.b) {

                                rightClaw.setPosition(RIGHT_CLAW_CLOSE);
                                leftClaw.setPosition(LEFT_CLAW_CLOSE);

                            }
                            if (gamepad2.right_bumper) {
                                if (!rightClawPressed) {
                                    rightClawPressed = true; // Set the flag to true when the bumper is initially pressed
                                    // Toggle the right claw position when the bumper is pressed
                                    if (rightClaw.getPosition() == RIGHT_CLAW_OPEN) {
                                        rightClaw.setPosition(RIGHT_CLAW_CLOSE);
                                    } else {
                                        rightClaw.setPosition(RIGHT_CLAW_OPEN);
                                    }
                                }
                            } else {
                                rightClawPressed = false; // Reset the flag when the bumper is released
                            }

                            // Check the left bumper on gamepad2 to control the left claw
                            if (gamepad2.left_bumper) {
                                if (!leftClawPressed) {
                                    leftClawPressed = true; // Set the flag to true when the bumper is initially pressed
                                    // Toggle the left claw position when the bumper is pressed
                                    if (leftClaw.getPosition() == LEFT_CLAW_OPEN) {
                                        leftClaw.setPosition(LEFT_CLAW_CLOSE);
                                    } else {
                                        leftClaw.setPosition(LEFT_CLAW_OPEN);
                                    }
                                }
                            } else {
                                leftClawPressed = false; // Reset the flag when the bumper is released
                            }
                            if (gamepad1.dpad_left) {

                                StateTime.reset();
                                state = State.UNDER_BAR;

                            }


                            break;

                        case HANG:

                            if (gamepad1.y) {
                                Subsystems.hangingArm(3900);
                            }

                            
                            if (gamepad1.x) {

                                Subsystems.hangingArm(200);
                                wrist.setPosition(WRIST_UP);

                            }

                            break;

                        case DRONE:

                            Subsystems.hangingArm(2500);

                            if (Math.abs(hangingMotor.getCurrentPosition()) >= 2490) {

                                drone.setPosition(1);

                            }

                            if (StateTime.time() > 5) {

                                StateTime.reset();
                                state = State.INITIAL;

                            }

                            break;

                        case UNDER_BAR:

                            Subsystems.slideAngle(ARM_RESTING);
                            Subsystems.slideExtension(0);
                            Subsystems.hangingArm(0);
                            wrist.setPosition(WRIST_PIXEL_PICKUP);
                            if (gamepad1.y) {

                                StateTime.reset();
                                state = State.HANG;

                            }
                            if (gamepad1.x) {

                                StateTime.reset();
                                state = State.INITIAL;

                            }
                            if (gamepad1.b) {

                                StateTime.reset();
                                state = State.PIXEL_PICKUP;

                            }
                            if (gamepad1.right_bumper) {

                                StateTime.reset();
                                state = State.BACKBOARD;

                            }

                            break;

                }

                    

                    double y = -gamepad2.left_stick_y; // Remember, y stick value is reversed
                    double x = gamepad2.left_stick_x * 1.1 ; // Counteract imperfect strafing
                    double rx = gamepad2.right_stick_x * 0.59;


                    double theta = Math.atan2(y, x);
                    double power = Math.hypot(x, y);
                    double sin = Math.sin(theta - Math.PI / 4);
                    double cos = Math.cos(theta - Math.PI / 4);
                    double max = Math.max(Math.abs(sin), Math.abs(cos));
                    double frontLeftPower = power * cos / max + rx;
                    double frontRightPower = power * sin / max - rx;
                    double backLeftPower = power * sin / max + rx;
                    double backRightPower = power * cos / max - rx;

                    if ((power + Math.abs(rx)) > 1) {
                        frontLeftPower /= power + rx;
                        frontRightPower /= power + rx;
                        backLeftPower /= power + rx;
                        backRightPower /= power + rx;
                    }

                    // Set motor powers
                    leftFront.setPower(frontLeftPower);
                    leftRear.setPower(backLeftPower);
                    rightFront.setPower(frontRightPower);
                    rightRear.setPower(backRightPower);



                }
            }
        }
    }


