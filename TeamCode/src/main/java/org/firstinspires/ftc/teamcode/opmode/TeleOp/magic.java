package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.common.hardware.hardwareinit.armmotor;
import static org.firstinspires.ftc.teamcode.common.hardware.hardwareinit.leftSlide;
import static org.firstinspires.ftc.teamcode.common.hardware.hardwareinit.rightSlide;
import static org.firstinspires.ftc.teamcode.common.hardware.hardwareinit.leftWrist;
import static org.firstinspires.ftc.teamcode.common.hardware.hardwareinit.rightWrist;


import org.firstinspires.ftc.teamcode.common.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.common.SampleMecanumDrive;

import java.lang.Math;

@TeleOp(name = "main", group = "org/firstinspires/ftc/teamcode/drive/opmode/Competition")
public class magic extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    public static DcMotorEx rightFront;
    public static DcMotorEx leftFront;
    public static DcMotorEx leftRear;
    public static DcMotorEx rightRear;
    //public static Servo drone;
    public static double h = .7;
    //public static Servo leftClaw;
    //public static Servo rightClaw;
    public static double k = 0.3;
    public static int c = 0;
    public static double b = 0.5;
    public static double s;
    public static PIDController controller;
    public static double p = 0.003, i = 0, d = 0.00015;
    public static double f = 0.08;
    public final double ticks_per_rev = 537.6;
    TouchSensor touch;
    double maxVelocity = 50; // Set your maximum desired velocity
    double maxAcceleration = 50; // Set your maximum desired acceleration
    double joystickThreshold = 0.1; // Set a threshold for joystick dead zone
    private static final long LOOP_PERIOD_MS = 10;

    double currentVelocityX = 0.0;
    double currentVelocityY = 0.0;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        touch = hardwareMap.get(TouchSensor.class, "touch");
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
        armmotor = hardwareMap.get(DcMotorEx.class, "armmotor");
        armmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //drone = hardwareMap.get(Servo.class, "drone");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        leftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftWrist = hardwareMap.get(Servo.class, "leftWrist");
        rightWrist = hardwareMap.get(Servo.class, "rightWrist");
        //leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        //rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        controller = new PIDController(p, i, d);
        telemetry.addData("posleftslide", leftSlide.getCurrentPosition());
        telemetry.addData("posrightslide", rightSlide.getCurrentPosition());
        telemetry.addData("posarmmotor", armmotor.getCurrentPosition());
        telemetry.update();
        Subsystem.initialize();


        //////////////////////////////////////////////////////////////////


        waitForStart();
        if (opModeIsActive()) {


            while (opModeIsActive()) {


                telemetry.addData("posleftslide", leftSlide.getCurrentPosition());
                telemetry.addData("posrightslide", rightSlide.getCurrentPosition());
                telemetry.addData("posarmmotor", armmotor.getCurrentPosition());
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("posleftWrist", leftWrist.getPosition());
                telemetry.addData("posrightWrist", rightWrist.getPosition());
                telemetry.update();

                if (gamepad1.right_bumper) {

                    Subsystem.syncedSlides(5000); //fully extended

                } else if (gamepad1.left_bumper) {

                    Subsystem.syncedSlides(50); //startpos

                } else if (gamepad1.dpad_up) {

                    //small adjustments to slides

                    leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + 100);
                    leftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    leftSlide.setPower(1);
                    rightSlide.setTargetPosition(rightSlide.getCurrentPosition() + 100);
                    rightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rightSlide.setPower(1);

                } else if (gamepad1.dpad_down) {

                    //small adjustments to slides

                    leftSlide.setTargetPosition(leftSlide.getCurrentPosition() - 100);
                    leftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    leftSlide.setPower(1);
                    rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - 100);
                    rightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rightSlide.setPower(1);

                } else if (gamepad1.dpad_left) {

                    armmotor.setTargetPosition(leftSlide.getCurrentPosition() + 100);
                    armmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    armmotor.setPower(1);

                } else if (gamepad1.dpad_right) {

                    armmotor.setTargetPosition(leftSlide.getCurrentPosition() - 100);
                    armmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    armmotor.setPower(1);

                } else if (gamepad1.y) {

                    if (rightSlide.getCurrentPosition() <= 100 ) {

                        Subsystem.armPosition(-3500); //hanging
                        Subsystem.syncedWrist(1);

                    }

                } else if (gamepad1.x) {

                    Subsystem.armPosition(-450); //backboard angle
                    Subsystem.syncedSlides(50); //startpos


                } else if (gamepad1.b) {

                    Subsystem.armPosition(-2450); //pixel angle
                    Subsystem.syncedSlides(4500); //slide action


                }else if (gamepad2.a) {

                    Subsystem.syncedWrist(0.4);

                } else if (gamepad2.dpad_left) {

                    Subsystem.syncedWrist(1);

                }

                if (touch.isPressed()) {

                    telemetry.addData("Touch Sensor", "Is Pressed");
                    telemetry.update();
                    rightSlide.setPower(0);
                    leftSlide.setPower(0);
                    rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                } else {

                    telemetry.addData("Touch Sensor", "Is Not Pressed");
                    telemetry.update();

                }



                //////////////////////////////////////////////////////////////

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


                ///////////////////////////////////////////////////////


                double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad2.left_stick_x; // Counteract imperfect strafing
                double rx = gamepad2.right_stick_x * 0.59;

/*
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = ((y + x + rx)) / denominator;
                double backLeftPower = ((y - x + rx)) / denominator;
                double frontRightPower = ((y - x - rx)) / denominator;
                double backRightPower = ((y + x - rx)) / denominator;
                */

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

                while ((Math.abs(gamepad1.left_stick_x) > joystickThreshold || Math.abs(gamepad1.left_stick_y) > joystickThreshold)) {
                    double targetVelocityX = gamepad1.left_stick_x * maxVelocity;
                    double targetVelocityY = gamepad1.left_stick_y * maxVelocity;

                    // Calculate acceleration (considering both acceleration and deceleration)
                    double accelerationX = (targetVelocityX - currentVelocityX) / Math.min(1.0, getRuntime());
                    double accelerationY = (targetVelocityY - currentVelocityY) / Math.min(1.0, getRuntime());

                    // Cap the accelerations to the maximum value
                    accelerationX = Range.clip(accelerationX, -maxAcceleration, maxAcceleration);
                    accelerationY = Range.clip(accelerationY, -maxAcceleration, maxAcceleration);

                    // Integrate accelerations to get velocities
                    currentVelocityX += accelerationX * (LOOP_PERIOD_MS / 1000.0);
                    currentVelocityY += accelerationY * (LOOP_PERIOD_MS / 1000.0);

                    // Cap the velocities to the maximum value
                    currentVelocityX = Range.clip(currentVelocityX, -maxVelocity, maxVelocity);
                    currentVelocityY = Range.clip(currentVelocityY, -maxVelocity, maxVelocity);

                    // Translate velocities to motor outputs (using your mecanum drivetrain code)

                    double frontLeftPower = currentVelocityY - currentVelocityX + rx;
                    double frontRightPower = currentVelocityY + currentVelocityX  - rx;
                    double backLeftPower = currentVelocityY + currentVelocityX + rx;
                    double backRightPower = currentVelocityY - currentVelocityX - rx;


                    double maxPower = Math.max(
                            Math.abs(frontLeftPower),
                            Math.abs(frontRightPower)
                    );

                    if (maxPower > 1.0) {
                        frontLeftPower /= maxPower;
                        frontRightPower /= maxPower;
                        backLeftPower /= maxPower;
                        backRightPower /= maxPower;
                    }

                    leftFront.setPower(frontLeftPower);
                    leftRear.setPower(backLeftPower);
                    rightFront.setPower(frontRightPower);
                    rightRear.setPower(backRightPower);


                }


/*
                for (double i = .5; i <= 1; i += 0.08) {

                    double h = i * 100;
                    i = h / 100;

                    Thread.sleep(1 / 100000000);

                    leftFront.setPower(frontLeftPower * i);
                    leftRear.setPower(backLeftPower * i);
                    rightFront.setPower(frontRightPower * i);
                    rightRear.setPower(backRightPower * i);
                }
*/

            }
        }
    }
}

