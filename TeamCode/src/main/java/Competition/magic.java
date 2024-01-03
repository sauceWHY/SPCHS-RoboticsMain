package Competition;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import static org.firstinspires.ftc.teamcode.hardwareinit.armmotor;
import static org.firstinspires.ftc.teamcode.hardwareinit.leftSlide;
import static org.firstinspires.ftc.teamcode.hardwareinit.rightSlide;


import org.firstinspires.ftc.teamcode.Subsystems;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.lang.Math;

@TeleOp(name = "main", group = "Competition")
public class magic extends LinearOpMode {

    public static DcMotorEx rightFront;
    public static DcMotorEx leftFront;
    public static DcMotorEx leftRear;
    public static DcMotorEx rightRear;
    public static Servo drone;
    public static double h = .7;
    public static Servo hand;
    //public static Servo leftClaw;
    //public static Servo rightClaw;
    //public static Servo leftWrist;
    //public static Servo rightWrist;
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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
        drone = hardwareMap.get(Servo.class, "drone");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        //leftWrist = hardwareMap.get(Servo.class, "left Wrist");
        //rightWrist = hardwareMap.get(Servo.class, "rightWrist");
        //leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        //rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        controller = new PIDController(p, i, d);
        telemetry.addData("posleftslide", leftSlide.getCurrentPosition());
        telemetry.addData("posrightslide", rightSlide.getCurrentPosition());
        telemetry.addData("posarmmotor", armmotor.getCurrentPosition());
        telemetry.update();
        Subsystems.initialize();



        //////////////////////////////////////////////////////////////////


        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {


                int startposright = 50;
                int startposleft = -50;
                int rightfullyex = 5700;
                int leftfullyex = -5700;
                int hangingarmpos = -4000;
                int backboardangle = -300;

                telemetry.addData("posleftslide", leftSlide.getCurrentPosition());
                telemetry.addData("posrightslide", rightSlide.getCurrentPosition());
                telemetry.addData("posarmmotor", armmotor.getCurrentPosition());
                telemetry.addData("x", drive.getPoseEstimate().getX());
                telemetry.addData("y", drive.getPoseEstimate().getY());
                telemetry.addData("heading", drive.getPoseEstimate().getHeading());
                telemetry.update();

                if (gamepad1.right_bumper) {

                    Subsystems.syncedSlides(5700);

                } else if (gamepad1.left_bumper) {

                    Subsystems.syncedSlides(50);

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

                    Subsystems.armPosition(-3500);

                } else if (gamepad1.x) {

                    Subsystems.armPosition(-450);

                } else if (gamepad1.b) {
                    Subsystems.armPosition(-2450);
                }/*else if (gamepad1.a) {
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
                }

                //////////////////////////////////////////////////////////////

                if (gamepad2.x){
                    c = c+1;
                    if (c == 1000){
                        drone.setPosition(0.9);
                    }
                } else {
                    drone.setPosition(0.65);
                    c = 0;
                }

                if (0.4 <= (h - (-gamepad1.left_stick_y/1000)) && (h - (-gamepad1.left_stick_y/1000)) <= 1){
                    h = h - (-gamepad1.left_stick_y/1000);
                }

                if (0.30 <= (b - (k/1000)) && (b - (k/1000)) <= 0.80){
                    b = b - (k/1000);
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
                double theta = Math.atan2(y,x);
                double power = Math.hypot(x,y);
                double sin = Math.sin(theta - Math.PI/4);
                double cos = Math.cos(theta - Math.PI/4);
                double max = Math.max(Math.abs(sin), Math.abs(cos));
                double frontLeftPower = power * cos/max + rx;
                double frontRightPower = power * sin/max - rx;
                double backLeftPower = power * sin/max + rx;
                double backRightPower = power * cos/max - rx;

                if ((power + Math.abs(rx)) > 1) {
                    frontLeftPower /= power + rx;
                    frontRightPower /= power + rx;
                    backLeftPower /= power + rx;
                    backRightPower /= power + rx;
                }


                for(double i = .5; i <= 1; i += 0.08) {

                    double h = i*100;
                    i = h/100;

                    Thread.sleep(1/100000000);

                    leftFront.setPower(frontLeftPower * i);
                    leftRear.setPower(backLeftPower * i);
                    rightFront.setPower(frontRightPower * i);
                    rightRear.setPower(backRightPower * i);
                }




            }
        }
    }


