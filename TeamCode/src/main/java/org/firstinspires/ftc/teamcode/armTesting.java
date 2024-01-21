package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotHardware.*;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class armTesting extends LinearOpMode {
    public static PIDController controller;
    public static double p = 0.003, i = 0, d = 0.00015;
    public static double f = 0.08;
    public final double ticks_per_rev = 537.6;


    public void runOpMode() throws InterruptedException{

        armmotor = hardwareMap.get(DcMotorEx.class, "armmotor");
        armmotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        leftSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        rightSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {

                if (gamepad1.a) {
                    Subsystems.armPosition(0);
                }
                if (gamepad1.b) {
                    Subsystems.armPosition(500);
                }
                if (gamepad1.y) {
                    Subsystems.armPosition(1000);
                }
                if (gamepad1.x) {
                    Subsystems.armPosition(1100);
                }
                if (gamepad1.dpad_up) {
                    Subsystems.armPosition(1200);
                }
                if (gamepad1.dpad_right) {
                    Subsystems.armPosition(1300);
                }
                if (gamepad1.dpad_down) {
                    Subsystems.armPosition(1400);
                }
                if (gamepad1.dpad_left) {
                    Subsystems.armPosition(1500);
                }
                if (gamepad1.right_bumper) {
                    Subsystems.armPosition(1600);
                }if (gamepad1.left_bumper) {
                    Subsystems.armPosition(1700);
                }
                if (gamepad1.right_trigger > 0.5) {
                    Subsystems.armPosition(1800);
                }
                if (gamepad1.left_trigger > 0.5) {
                    Subsystems.armPosition(1900);
                }
                if (gamepad2.a) {
                    Subsystems.armPosition(2000);
                }
                if (gamepad2.b) {
                    Subsystems.syncedSlides(0);
                }
                if (gamepad2.y) {
                    Subsystems.syncedSlides(3500);
                }
                if (gamepad2.x) {
                    Subsystems.syncedSlides(3700);
                }
                if (gamepad2.dpad_left) {
                    Subsystems.syncedSlides(3900);
                }
                if (gamepad2.dpad_down) {
                    Subsystems.syncedSlides(4100);
                }
                if (gamepad2.dpad_right) {
                    Subsystems.syncedSlides(4300);
                }
                if (gamepad2.dpad_up) {
                    Subsystems.syncedSlides(4500);
                }



            }
        }

    }
}
