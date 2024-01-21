package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import static org.firstinspires.ftc.teamcode.RobotHardware.*;


@TeleOp
public class drivetrain extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    public static PIDController controller;
    public static double p = 0.003, i = 0, d = 0.00015;
    public static double f = 0.08;
    public final double ticks_per_rev = 537.6;
    double Kp = 0.015;
    static boolean pressed = false;
    double error;
    double botHeading;
    double target = 0;


    public void runOpMode() {

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
        double botHeading = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle;
        if (gamepad1.right_stick_x != 0) {
            target = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle;

        }

        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {



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

            }
        }

    }
}
