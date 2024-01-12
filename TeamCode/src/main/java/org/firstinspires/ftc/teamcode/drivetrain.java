package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.Timer;
import java.util.TimerTask;

@TeleOp
public class drivetrain extends LinearOpMode {
    public static DcMotorEx rightFront;
    public static DcMotorEx leftFront;
    public static DcMotorEx leftRear;
    public static DcMotorEx rightRear;
    final Timer timer = new Timer();

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
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        waitForStart();
        if (opModeIsActive()) {
            TimerTask arm = new TimerTask() {
                @Override
                public void run() {
                    drive.turn(Math.toRadians(90));
                }
            };
            TimerTask slides = new TimerTask() {
                @Override
                public void run() {
                    drive.turn(Math.toRadians(180));
                }
            };
            TimerTask claw = new TimerTask() {
                @Override
                public void run() {
                    drive.turn(Math.toRadians(270));
                }
            };


            while (opModeIsActive()) {

                telemetry.addData("heading", drive.getExternalHeading());
                telemetry.update();

                if (gamepad2.a) {
                    drive.turn(Math.toRadians(90));
                    Thread.sleep(1000);
                    drive.turn(Math.toRadians(90));

                }


                double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad2.left_stick_x; // Counteract imperfect strafing
                double rx = gamepad2.right_stick_x * 0.75;
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


                for (double i = .9; i <= 1; i += 0.04) {

                    double h = i * 100;
                    i = h / 100;

                    Thread.sleep(1 / 100000000);

                    leftFront.setPower(frontLeftPower * i);
                    leftRear.setPower(backLeftPower * i);
                    rightFront.setPower(frontRightPower * i);
                    rightRear.setPower(backRightPower * i);
                }

            }
        }

    }
}
