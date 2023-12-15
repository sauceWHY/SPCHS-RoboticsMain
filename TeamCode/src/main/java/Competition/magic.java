package Competition;

import static org.opencv.core.Core.sqrt;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PIDArm;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.lang.Math;

@TeleOp(name = "main", group = "Competition")
public class magic extends LinearOpMode {

    public static DcMotor rightFront;
    public static DcMotor leftFront;
    public static DcMotor leftRear;
    public static DcMotor rightRear;
    public static DcMotor armmotor;
    public static Servo drone;
    public static Servo arm;
    public static DcMotor intakemotor;
    public static DcMotorEx leftSlide;
    public static DcMotorEx rightSlide;
    public static double h = .7;
    public static Servo hand;
    public static Servo leftClaw;
    public static Servo rightClaw;
    public static Servo leftWrist;
    public static Servo rightWrist;
    public static double k = 0.3;
    public static int c = 0;
    public static double b = 0.5;
    public static double s;
    public static PIDController controller;
    public static double p=0.003, i=0, d=0.00015;
    public static double f = 0.08;
    public final double ticks_per_rev = 537.6;


    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armmotor = hardwareMap.get(DcMotor.class, "armmotor");
        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        drone = hardwareMap.get(Servo.class, "drone");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftWrist = hardwareMap.get(Servo.class, "left Wrist");
        rightWrist = hardwareMap.get(Servo.class, "rightWrist");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        controller = new PIDController(p, i, d);



        //////////////////////////////////////////////////////////////////


        waitForStart();
        if (opModeIsActive()) {
            drone.setPosition(0);

            while (opModeIsActive()) {


                int startpos1 = 50;
                int startpos2 = -50;
                int rightfullyex = 5700;
                int leftfullyex = -5700;
                int hangingarmpos = -4000;
                int backboardangle = -300;

                telemetry.addData("posleftslide", leftSlide.getCurrentPosition());
                telemetry.addData("posrightslide", rightSlide.getCurrentPosition());
                telemetry.addData("posarmmotor", armmotor.getCurrentPosition());
                telemetry.update();

                if (gamepad1.right_bumper) {
                    //if (leftSlide.getCurrentPosition() = ) {
                        leftSlide.setTargetPosition(leftfullyex);
                        rightSlide.setTargetPosition(rightfullyex);
                        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        controller.setPID(p, i, d);
                        int slidePos = leftSlide.getCurrentPosition();
                        double pid = controller.calculate(slidePos, leftfullyex);
                        double ff = Math.cos(Math.toRadians(leftfullyex / ticks_per_rev)) * f;

                        double power = pid + ff;

                        leftSlide.setPower(power);
                        rightSlide.setPower(power);
                   // }
                } else if (gamepad1.left_bumper) {
                    leftSlide.setTargetPosition(startpos2);
                    rightSlide.setTargetPosition(startpos1);
                    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    controller.setPID(p, i, d);
                    int slidePos = leftSlide.getCurrentPosition();
                    double pid = controller.calculate(slidePos, startpos2);
                    double ff = Math.cos(Math.toRadians(startpos2 / ticks_per_rev)) * f;

                    double power = pid + ff;

                    leftSlide.setPower(power);
                    rightSlide.setPower(power);
                } else if (gamepad1.dpad_up) {
                    leftSlide.setTargetPosition(leftSlide.getCurrentPosition() - 100);
                    leftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    leftSlide.setPower(1);
                    rightSlide.setTargetPosition(rightSlide.getCurrentPosition() + 100);
                    rightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rightSlide.setPower(1);
                } else if (gamepad1.dpad_down) {
                    leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + 100);
                    leftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    leftSlide.setPower(1);
                    rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - 100);
                    rightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rightSlide.setPower(1);
                } else if (gamepad1.y) {
                    armmotor.setTargetPosition(hangingarmpos);
                    armmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    controller.setPID(p, i, d);
                    int slidePos = armmotor.getCurrentPosition();
                    double pid = controller.calculate(slidePos, hangingarmpos);
                    double ff = Math.cos(Math.toRadians(hangingarmpos / ticks_per_rev)) * f;

                    double power = pid + ff;

                    armmotor.setPower(power);
                } else if (gamepad1.x) {
                    armmotor.setTargetPosition(backboardangle);
                    armmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    controller.setPID(p, i, d);
                    int slidePos = armmotor.getCurrentPosition();
                    double pid = controller.calculate(slidePos, backboardangle);
                    double ff = Math.cos(Math.toRadians(backboardangle / ticks_per_rev)) * f;

                    double power = pid + ff;

                    armmotor.setPower(power);
                } /*else if (gamepad1.a) {
                    if (leftClaw.getPosition() == 1) {
                        leftClaw.setPosition(0);
                        Thread.sleep(750);
                    } else if (leftClaw.getPosition() == 0) {
                        leftClaw.setPosition(1);
                    }
                } else if (gamepad1.b){
                    if (rightClaw.getPosition() == 1) {
                        rightClaw.setPosition(0);
                        Thread.sleep(750);
                    } else if (rightClaw.getPosition() == 0) {
                        rightClaw.setPosition(1);
                        Thread.sleep(750);
                    }
*/

                //////////////////////////////////////////////////////////////

                if (0.4 <= (h - (-gamepad1.left_stick_y/1000)) && (h - (-gamepad1.left_stick_y/1000)) <= 1){
                    h = h - (-gamepad1.left_stick_y/1000);
                }


                ///////////////////////////////////////////////////////


                double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad2.left_stick_x; // Counteract imperfect strafing
                double rx = gamepad2.right_stick_x * 0.59;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = ((y + x + rx)) / denominator;
                double backLeftPower = ((y - x + rx)) / denominator;
                double frontRightPower = ((y - x - rx)) / denominator;
                double backRightPower = ((y + x - rx)) / denominator;




                for(double i = .5; i <= 1; i += 0.08) {

                    double h = i*100;
                    i = h/100;

                    Thread.sleep(1/100000000);

                    leftFront.setPower(frontLeftPower * i);
                    leftRear.setPower(backLeftPower * i);
                    rightFront.setPower(frontRightPower * i);
                    rightRear.setPower(backRightPower * i);
                }

                if (gamepad2.x){
                    c = c+1;
                    if (c == 1000){
                        drone.setPosition(0.9);
                    }
                } else {
                    drone.setPosition(0.65);
                    c = 0;
                }
                if (0.30 <= (b - (k/1000)) && (b - (k/1000)) <= 0.80){
                    b = b - (k/1000);
                }
                arm.setPosition(h);
                hand.setPosition(k);
            }
        }
    }
}


