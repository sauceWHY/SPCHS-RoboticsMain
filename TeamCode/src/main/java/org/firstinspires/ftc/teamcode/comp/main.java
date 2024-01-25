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
    import org.firstinspires.ftc.teamcode.Subsystems;
    import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
    import static org.firstinspires.ftc.teamcode.Robot.*;


    @TeleOp(name = "main", group = "Competition")
    public class main extends LinearOpMode {

        private enum State {
            INITIAL,
            PIXEL_PICKUP,
            SLIDE_EXTENDED,
            BACKBOARD,
            HANG,
            DRONE,
            DONE
        }
        public final ElapsedTime runtime = new ElapsedTime();
        private final ElapsedTime StateTime = new ElapsedTime();
        public static PIDController controller;
        public static double p = 0.003, i = 0, d = 0.00015;
        public static double f = 0.08;
        public final double ticks_per_rev = 537.6;
        static boolean pressed = false;
        private State CurrentState;
        private static final int PIXEL_ARM_ANGLE = 3000;
        private static final int BACK_BOARD_ANGLE = 0;
        private static final int SLIDE_EXTENDED = 1800;
        private static final int SLIDE_START_POS = 0;





        @Override
        public void runOpMode() throws InterruptedException {

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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
            wrist.setPosition(0);
            leftClaw = hardwareMap.get(Servo.class, "leftClaw");
            rightClaw = hardwareMap.get(Servo.class, "rightClaw");
            rightClaw.setPosition(0.6);
            leftClaw.setPosition(0.17);

            //Sensor
            touch = hardwareMap.get(TouchSensor.class, "touch");

            IMU imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
            imu.initialize(parameters);
            controller = new PIDController(p, i, d);
            Subsystems.init();
            Pose2d poseEstimate = drive.getPoseEstimate();




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




            waitForStart();
            if (opModeIsActive()) {

                runtime.reset();
                State state = State.BACKBOARD;
                StateTime.reset();

                while (opModeIsActive()) {
                    telemetry.addData("Status", "Run Time: " + runtime);
                    telemetry.addData("posslide", slideExtension.getCurrentPosition());
                    telemetry.addData("poshangingMotor", hangingMotor.getCurrentPosition());
                    telemetry.addData("posslidePivot", slidePivot.getCurrentPosition());
                    telemetry.addData("posleftWrist", wrist.getPosition());
                    telemetry.addData("posrightClaw", rightClaw.getPosition());
                    telemetry.addData("posleftClaw", leftClaw.getPosition());
                    telemetry.addData("armVoltage", slidePivot.getCurrent(CurrentUnit.AMPS));
                    telemetry.addData("slideVoltage", slideExtension.getCurrent(CurrentUnit.AMPS));
                    telemetry.addData("hangingVoltage", hangingMotor.getCurrent(CurrentUnit.AMPS));
                    telemetry.update();

                    if (gamepad1.right_trigger > 0.5 && gamepad1.left_trigger > 0.5) {

                        StateTime.reset();
                        state = State.BACKBOARD;

                    }

                    switch (state) {

                        case BACKBOARD:

                            Subsystems.slideAngle(0);
                            Subsystems.slideExtension(0);

                            if (slideExtension.getCurrentPosition() <= 60) {
                                wrist.setPosition(0);
                            }

                            if (gamepad1.right_bumper && Math.abs(slidePivot.getCurrentPosition() - BACK_BOARD_ANGLE) <= 10 && Math.abs(slideExtension.getCurrentPosition() - SLIDE_START_POS) <= 60) {

                                StateTime.reset();
                                state = State.SLIDE_EXTENDED;

                            }
                            if (gamepad1.b && Math.abs(slidePivot.getCurrentPosition() - BACK_BOARD_ANGLE) <= 10 && Math.abs(slideExtension.getCurrentPosition() - SLIDE_START_POS) <= 60) {

                                StateTime.reset();
                                state = State.PIXEL_PICKUP;

                            }
                            if (gamepad1.y && Math.abs(slidePivot.getCurrentPosition() - BACK_BOARD_ANGLE) <= 10 && Math.abs(slideExtension.getCurrentPosition() - SLIDE_START_POS) <= 60) {

                                StateTime.reset();
                                state = State.HANG;

                            }
                            if (gamepad2.y && Math.abs(slidePivot.getCurrentPosition() - BACK_BOARD_ANGLE) <= 10 && Math.abs(slideExtension.getCurrentPosition() - SLIDE_START_POS) <= 60) {

                                StateTime.reset();
                                state = State.DRONE;

                            }
                            break;

                        case SLIDE_EXTENDED:

                            Subsystems.slideExtension(1900);

                            if (gamepad1.left_bumper) {

                                StateTime.reset();
                                state = State.BACKBOARD;

                            }
                            if (gamepad1.dpad_up) {

                                slideExtension.setTargetPosition(slideExtension.getCurrentPosition() + 100);
                                slideExtension.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                                slideExtension.setPower(1);

                            }
                            if (gamepad1.dpad_down) {

                                slideExtension.setTargetPosition(slideExtension.getCurrentPosition() - 100);
                                slideExtension.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                                slideExtension.setPower(1);

                            }
                            if (gamepad2.a) {

                                rightClaw.setPosition(0.3);
                                leftClaw.setPosition(0.6);

                            }
                            if (gamepad2.b) {

                                rightClaw.setPosition(0.6);
                                leftClaw.setPosition(0.17);

                            }
                            if (gamepad2.right_bumper) {

                                rightClaw.setPosition(0.6);

                            }
                            if (gamepad2.left_bumper) {

                                leftClaw.setPosition(0.17);

                            }

                            break;
                        case PIXEL_PICKUP:

                            Subsystems.slideAngle(PIXEL_ARM_ANGLE);
                            wrist.setPosition(0.6);


                            if (StateTime.time() > 1) {

                                Subsystems.slideExtension(SLIDE_EXTENDED);

                            }

                            if (gamepad1.x && Math.abs(slideExtension.getCurrentPosition()) >= 1500) {

                                StateTime.reset();
                                state = State.BACKBOARD;

                            }
                            if (gamepad2.a) {

                                rightClaw.setPosition(0.3);
                                leftClaw.setPosition(0.55);

                            }
                            if (gamepad2.b) {

                                rightClaw.setPosition(0.62);
                                leftClaw.setPosition(0.16);

                            }
                            if (gamepad2.right_bumper) {

                                rightClaw.setPosition(0.6);

                            }
                            if (gamepad2.left_bumper) {

                                leftClaw.setPosition(0.17);

                            }


                            break;

                        case HANG:

                            Subsystems.hangingArm(3900);
                            if (gamepad1.y) {
                                Subsystems.hangingArm(3900);
                            }
                            if (gamepad1.a) {
                                Subsystems.hangingArm(300);

                            }
                            
                            if (gamepad1.x) {

                                StateTime.reset();
                                state = State.BACKBOARD;

                            }

                            break;

                        case DRONE:

                            if (StateTime.time() > 1) {

                                StateTime.reset();
                                state = State.BACKBOARD;

                            }

                            break;
                        case DONE:

                            break;

                }
                    if (touch.isPressed() && !pressed) {

                        telemetry.addData("Touch Sensor", "Is Pressed");
                        telemetry.update();
                        if (slideExtension.getPower() <= 0) {
                            slideExtension.setPower(0);
                        }
                        slideExtension.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        pressed = true;

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


