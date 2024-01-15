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


    import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
    import org.firstinspires.ftc.teamcode.hardware;

    import static org.firstinspires.ftc.teamcode.hardware.*;
    import java.lang.Math;

    import org.firstinspires.ftc.teamcode.Subsystems;


    @TeleOp(name = "main", group = "Competition")
    public class main extends LinearOpMode {
        private final ElapsedTime runtime = new ElapsedTime();
        public static PIDController controller;
        public static double p = 0.003, i = 0, d = 0.00015;
        public static double f = 0.08;
        public final double ticks_per_rev = 537.6;
        TouchSensor touch;
        double maxVelocity = 50; // Set your maximum desired velocity
        double maxAcceleration = 50; // Set your maximum desired acceleration
        double currentVelocityX = 0.0;
        double currentVelocityY = 0.0;
        double Kp = 0.01;

        static boolean pressed = false;

        /**
         * This function is executed when this OpMode is selected from the Driver Station.
         */
        @Override
        public void runOpMode() throws InterruptedException {

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            touch = hardwareMap.get(TouchSensor.class, "touch");
            IMU imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
            imu.initialize(parameters);
            controller = new PIDController(p, i, d);
            hardware.robotInit();
            Subsystems.init();


            //////////////////////////////////////////////////////////////////


            waitForStart();
            if (opModeIsActive()) {


                while (opModeIsActive()) {

                    telemetry.addData("Status", "Run Time: " + runtime.toString());
                    telemetry.addData("posleftslide", leftSlide.getCurrentPosition());
                    telemetry.addData("posrightslide", rightSlide.getCurrentPosition());
                    telemetry.addData("posarmmotor", armmotor.getCurrentPosition());
                    telemetry.addData("posleftWrist", leftWrist.getPosition());
                    telemetry.addData("posrightWrist", rightWrist.getPosition());
                    telemetry.addData("posrightClaw", rightClaw.getPosition());
                    telemetry.addData("posleftClaw", leftClaw.getPosition());
                    telemetry.update();

                    if (gamepad1.right_bumper) {

                        Subsystems.syncedSlides(4500); //fully extended
                        Subsystems.syncedWrist(1);

                    }
                    if (gamepad1.left_bumper) {

                        Subsystems.syncedSlides(50); //startpos
                        Subsystems.syncedWrist(0.4);

                    }
                    if (gamepad1.dpad_up) {

                        //small adjustments to slides

                        leftSlide.setTargetPosition(leftSlide.getCurrentPosition() - 100);
                        leftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(1);
                        rightSlide.setTargetPosition(rightSlide.getCurrentPosition() + 100);
                        rightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        rightSlide.setPower(1);

                    }
                    if (gamepad1.dpad_down) {

                        //small adjustments to slides

                        leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + 100);
                        leftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        leftSlide.setPower(1);
                        rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - 100);
                        rightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        rightSlide.setPower(1);

                    }
                    if (gamepad1.y) {

                        if (rightSlide.getCurrentPosition() <= 100) {

                            Subsystems.armPosition(1250); //hanging
                            Subsystems.syncedWrist(1);

                        }

                    }
                    if (gamepad1.x) {

                        Subsystems.armPosition(0); //backboard angle
                        Subsystems.syncedSlides(50); //startpos
                        Subsystems.syncedWrist(0.4);


                    }
                    if (gamepad1.b) {

                        Subsystems.armPosition(1700); //pixel angle
                        Subsystems.syncedSlides(4450); //slide action
                        Subsystems.syncedWrist(1);

                    }

                    if (gamepad2.a) {

                        rightClaw.setPosition(0.2);
                        leftClaw.setPosition(0.5);

                    }
                    if (gamepad2.b) {

                        rightClaw.setPosition(0.70);
                        leftClaw.setPosition(0.09);

                    }
                    if (gamepad2.left_bumper) {

                        leftClaw.setPosition(0.12);

                    }
                    if (gamepad2.right_bumper) {

                        rightClaw.setPosition(0.62);

                    }


                    if (touch.isPressed() && !pressed) {

                        telemetry.addData("Touch Sensor", "Is Pressed");
                        telemetry.update();
                        if (rightSlide.getPower() <= 0) {
                            rightSlide.setPower(0);
                        }
                        if (leftSlide.getPower() <= 0) {
                            leftSlide.setPower(0);
                        }
                        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        pressed = true;

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




/*
                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double frontLeftPower = ((y + x + rx)) / denominator;
                    double backLeftPower = ((y - x + rx)) / denominator;
                    double frontRightPower = ((y - x - rx)) / denominator;
                    double backRightPower = ((y + x - rx)) / denominator;


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

                    double y = -gamepad2.left_stick_y; // Remember, y stick value is reversed
                    double x = gamepad2.left_stick_x; // Counteract imperfect strafing
                    double rx = gamepad2.right_stick_x * 0.59;

                    double targetPositionX = motionProfile(maxAcceleration, maxVelocity, x, getRuntime());
                    double targetPositionY = motionProfile(maxAcceleration, maxVelocity, y, getRuntime());

                    // Calculate proportional control error
                    double errorX = targetPositionX - currentVelocityX;
                    double errorY = targetPositionY - currentVelocityY;

                    // Calculate motor powers with proportional control
                    double frontLeftPower = errorY + errorX + rx + Kp * errorX;
                    double backLeftPower = errorY - errorX + rx + Kp * errorX;
                    double frontRightPower = errorY - errorX - rx + Kp * errorX;
                    double backRightPower = errorY + errorX - rx + Kp * errorX;

                    // Update current velocities
                    currentVelocityX = leftFront.getVelocity();
                    currentVelocityY = leftRear.getVelocity();

                    // Set motor powers
                    leftFront.setPower(frontLeftPower);
                    leftRear.setPower(backLeftPower);
                    rightFront.setPower(frontRightPower);
                    rightRear.setPower(backRightPower);


                }
            }
        }

        double motionProfile(double maxAcceleration, double maxVelocity, double distance, double elapsed_time) {
            // Calculate the time it takes to accelerate to max velocity
            double acceleration_dt = maxVelocity / maxAcceleration;

            // If we can't accelerate to max velocity in the given distance, accelerate as much as possible
            double halfway_distance = distance / 2;
            double acceleration_distance = 0.5 * maxAcceleration * Math.pow(acceleration_dt, 2);

            if (acceleration_distance > halfway_distance) {
                acceleration_dt = Math.sqrt(halfway_distance / (0.5 * maxAcceleration));
            }

            acceleration_distance = 0.5 * maxAcceleration * Math.pow(acceleration_dt, 2);

            // Recalculate max velocity based on the time to accelerate and decelerate
            maxVelocity = maxAcceleration * acceleration_dt;

            // Decelerate at the same rate as accelerating
            double deceleration_dt = acceleration_dt;

            // Calculate the time at max velocity
            double cruise_distance = distance - 2 * acceleration_distance;
            double cruise_dt = cruise_distance / maxVelocity;
            double deceleration_time = acceleration_dt + cruise_dt;

            // Check if we're still in the motion profile
            double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
            if (elapsed_time > entire_dt) {
                return distance;
            }

            // If we're accelerating
            if (elapsed_time < acceleration_dt) {
                // Use the kinematic equation for acceleration
                return 0.5 * maxAcceleration * Math.pow(elapsed_time, 2);
            }

            // If we're cruising
            else if (elapsed_time < deceleration_time) {
                acceleration_distance = 0.5 * maxAcceleration * Math.pow(acceleration_dt, 2);
                double cruise_current_dt = elapsed_time - acceleration_dt;

                // Use the kinematic equation for constant velocity
                return acceleration_distance + maxVelocity * cruise_current_dt;
            }

            // If we're decelerating
            else {
                acceleration_distance = 0.5 * maxAcceleration * Math.pow(acceleration_dt, 2);
                cruise_distance = maxVelocity * cruise_dt;
                deceleration_time = elapsed_time - deceleration_time;

                // Use the kinematic equations to calculate the instantaneous desired position
                return acceleration_distance + cruise_distance + maxVelocity * deceleration_time
                        - 0.5 * maxAcceleration * Math.pow(deceleration_time, 2);
            }

        }
    }


