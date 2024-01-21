    package org.firstinspires.ftc.teamcode.comp;


    import com.arcrobotics.ftclib.controller.PIDController;
    import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.IMU;
    import com.qualcomm.robotcore.hardware.TouchSensor;
    import com.qualcomm.robotcore.util.ElapsedTime;


    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
    import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


    import java.lang.Math;

    import static org.firstinspires.ftc.teamcode.RobotHardware.*;
    import org.firstinspires.ftc.teamcode.Subsystems;


    @TeleOp(name = "main", group = "Competition")
    public class main extends LinearOpMode {

        private enum State {
            INITIAL,
            PIXEL_PICKUP,
            SLIDE_EXTENDED,
            GETTING_UNDER,
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
        double Kp = 0.015;

        static boolean pressed = false;
        double error;
        double botHeading;
        double target = 0;
        private State CurrentState;
        private static final int PIXEL_ARM_ANGLE = 1700;
        private static final int BACK_BOARD_ANGLE = 0;
        private static final int SLIDE_EXTENDED = 4400;
        private static final int SLIDE_START_POS = 50;
        double minSafeVoltage = 6.5;






        @Override
        public void runOpMode() throws InterruptedException {

            initializeHardware();

            IMU imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
            imu.initialize(parameters);
            controller = new PIDController(p, i, d);
            init();
            Subsystems.init();
            double botHeading = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle;
            if (gamepad1.right_stick_x != 0) {

                target = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle;

            }



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




            waitForStart();
            if (opModeIsActive()) {

                runtime.reset();
                StateTime.reset();
                State state = State.INITIAL;

                while (opModeIsActive()) {

                    telemetry.addData("Status", "Run Time: " + runtime.toString());
                    telemetry.addData("posleftslide", leftSlide.getCurrentPosition());
                    telemetry.addData("posrightslide", rightSlide.getCurrentPosition());
                    telemetry.addData("posarmmotor", armmotor.getCurrentPosition());
                    telemetry.addData("posleftWrist", leftWrist.getPosition());
                    telemetry.addData("posrightWrist", rightWrist.getPosition());
                    telemetry.addData("posrightClaw", rightClaw.getPosition());
                    telemetry.addData("posleftClaw", leftClaw.getPosition());
                    telemetry.addData("armVoltage", armmotor.getCurrent());
                    telemetry.addData("hangingVoltage", leftSlide.getCurrent());
                    telemetry.addData("slideVoltage", rightSlide.getCurrent());

                    if (gamepad1.right_trigger > 0.5 && gamepad1.left_trigger > 0.5) {

                        StateTime.reset();
                        state = State.BACKBOARD

                    }
                    if (getBatteryVoltage() < minSafeVoltage) {

                        telemetry.addData("Low Battery", "Voltage below safe level. Failsafe activated.");
                        telemetry.update();
                        StateTime.reset();
                        state = State.BACKBOARD

                    }

                    switch (CurrentState) {

                        case INITIAL:
                            rightClaw.setPosition(0.70);
                            leftClaw.setPosition(0.09);
                            state = State.BACKBOARD;
                            break;

                        case BACKBOARD:

                            Subsystems.armPosition(0);
                            Subsystems.syncedSlides(0);

                            if (gamepad1.right_bumper && Math.abs(armmotor.getCurrentPosition() - BACK_BOARD_ANGLE) <= 10 && Math.abs(rightSlide.getCurrentPosition() - SLIDE_START_POS) <= 60) {

                                StateTime.reset();
                                state = State.SLIDE_EXTENDED;

                            }
                            if (gamepad1.b && Math.abs(armmotor.getCurrentPosition() - BACK_BOARD_ANGLE) <= 10 && Math.abs(rightSlide.getCurrentPosition() - SLIDE_START_POS) <= 60) {

                                StateTime.reset();
                                state = State.PIXEL_PICKUP;

                            }
                            if (gamepad1.y && Math.abs(armmotor.getCurrentPosition() - BACK_BOARD_ANGLE) <= 10 && Math.abs(rightSlide.getCurrentPosition() - SLIDE_START_POS) <= 60) {

                                StateTime.reset();
                                state = State.HANG;

                            }
                            if (gamepad2.y && Math.abs(armmotor.getCurrentPosition() - BACK_BOARD_ANGLE) <= 10 && Math.abs(rightSlide.getCurrentPosition() - SLIDE_START_POS) <= 60) {

                                StateTime.reset();
                                state = State.DRONE;

                            }
                            if (gamepad1.dpad_left && Math.abs(armmotor.getCurrentPosition() - BACK_BOARD_ANGLE) <= 10 && Math.abs(rightSlide.getCurrentPosition() - SLIDE_START_POS) <= 60) {

                                StateTime.reset();
                                state = State.GETTING_UNDER;

                            }

                            break;

                        case SLIDE_EXTENDED:

                            Subsystems.slideRightPosition(4400);

                            if (gamepad1.left_bumper) {

                                StateTime.reset();
                                state = State.BACKBOARD;

                            }
                            if (gamepad1.dpad_up) {

                                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() + 100);
                                rightSlide.setPower(1);

                            }
                            if (gamepad1.dpad_down) {

                                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - 100);
                                rightSlide.setPower(1);

                            }

                            break;
                        case PIXEL_PICKUP:

                            Subsystems.armPosition(1200);

                            if (StateTime.time() > 0.2) {

                                Subsystems.slideRightPosition(SLIDE_EXTENDED);

                            }
                            if (StateTime.time() > 0.5) {

                                Subsystems.syncedWrist(0.95);

                            }
                            if (gamepad1.x && Math.abs(rightSlide.getCurrentPosition()) >= 3500) {

                                StateTime.reset();
                                state = State.BACKBOARD;

                            }
                            if (gamepad2.a) {

                                rightClaw.setPosition(0.2);
                                leftClaw.setPosition(0.5);

                            }
                            if (gamepad2.b) {

                                rightClaw.setPosition(0.70);
                                leftClaw.setPosition(0.09);

                            }
                            if (gamepad2.right_bumper) {

                                rightClaw.setPosition(0.62);

                            }
                            if (gamepad2.left_bumper) {

                                leftClaw.setPosition(0.12);

                            }


                            break;

                        case HANG:

                            Subsystems.slideLeftPosition(3000);

                            if (gamepad1.y) {

                                Subsystems.slideLeftPosition(500);

                            }
                            if (gamepad1.x) {

                                StateTime.reset();
                                state = State.BACKBOARD;

                            }

                            break;

                        case DRONE:

                            if (StateTime.time() > 1) {

                                StateTime.reset();
                                state = State.BACKBOARD

                            }

                            break;
                        case DONE:

                            break;

                }
                    if (touch.isPressed() && !pressed) {

                        telemetry.addData("Touch Sensor", "Is Pressed");
                        telemetry.update();
                        if (rightSlide.getPower() <= 0) {
                            rightSlide.setPower(0);
                        }
                        rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
\                        pressed = true;

                    }


                    //////////////////////////////////////////////////////////////
/*
                    if (gamepad2.x) {
                        c = c + 1;
                        if (c == 1000) {
                           // drone.setPosition(0.9);
                        }
                    } else {
                        // drone.setPosition(0.65);
                        c = 0;
                    }

                    if (0.4 <= (h - (-gamepad1.left_stick_y / 1000)) && (h - (-gamepad1.left_stick_y / 1000)) <= 1) {
                        h = h - (-gamepad1.left_stick_y / 1000);
                    }

                    if (0.30 <= (b - (k / 1000)) && (b - (k / 1000)) <= 0.80) {
                        b = b - (k / 1000);
                    }
                    //hand.setPosition(k);
                    //arm.setPosition(h);


 */

                    ///////////////////////////////////////////////////////







                    double y = -gamepad2.left_stick_y; // Remember, y stick value is reversed
                    double x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
                    double rx = gamepad2.right_stick_x * 0.59;

                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double frontLeftPower = ((y + x + rx)) / denominator;
                    double backLeftPower = ((y - x + rx)) / denominator;
                    double frontRightPower = ((y - x - rx)) / denominator;
                    double backRightPower = ((y + x - rx)) / denominator;

                    error = -Math.toDegrees(botHeading) + Math.toDegrees(target);
                    frontLeftPower += error * Kp;
                    backLeftPower += error * Kp;
                    frontRightPower += -error * Kp;
                    backRightPower += -error * Kp;

                    // Set motor powers
                    leftFront.setPower(frontLeftPower);
                    leftRear.setPower(backLeftPower);
                    rightFront.setPower(frontRightPower);
                    rightRear.setPower(backRightPower);



                    /*
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
*/



                }
            }
        }
    }


