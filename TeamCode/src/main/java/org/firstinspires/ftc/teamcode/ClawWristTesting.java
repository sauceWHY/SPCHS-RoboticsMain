package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import static org.firstinspires.ftc.teamcode.RobotHardware.*;


public class ClawWristTesting extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    public static PIDController controller;
    public static double p = 0.003, i = 0, d = 0.00015;
    public static double f = 0.08;
    public final double ticks_per_rev = 537.6;


    public void runOpMode() throws InterruptedException {

        leftWrist = hardwareMap.get(Servo.class, "leftWrist");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightWrist = hardwareMap.get(Servo.class, "rightWrist");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);


        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {

                if (gamepad1.a) {
                    Subsystems.syncedWrist(0.1);
                }
                if (gamepad1.b) {
                    Subsystems.syncedWrist(0.2);

                }
                if (gamepad1.y) {
                    Subsystems.syncedWrist(0.3);

                }
                if (gamepad1.x) {
                    Subsystems.syncedWrist(0.4);

                }
                if (gamepad1.dpad_left) {
                    Subsystems.syncedWrist(0.5);

                }
                if (gamepad1.dpad_down) {
                    Subsystems.syncedWrist(0.6);

                }
                if (gamepad1.dpad_right) {
                    Subsystems.syncedWrist(0.7);

                }
                if (gamepad1.dpad_up) {
                    Subsystems.syncedWrist(0.8);

                }
                if (gamepad1.right_bumper) {
                    Subsystems.syncedWrist(0.9);

                }
                if (gamepad1.left_bumper) {
                    Subsystems.syncedWrist(1);

                }
                if (gamepad2.a) {

                    leftClaw.setPosition(0.1);

                }
                if (gamepad2.b) {

                    leftClaw.setPosition(0.2);

                }
                if (gamepad2.y) {
                    leftClaw.setPosition(0.3);

                }
                if (gamepad2.x) {
                    leftClaw.setPosition(0.4);

                }
                if (gamepad2.dpad_left) {
                    leftClaw.setPosition(0.5);

                }
                if (gamepad2.dpad_down) {
                    leftClaw.setPosition(0.6);

                }
                if (gamepad2.dpad_right) {
                    leftClaw.setPosition(0.7);

                }
                if (gamepad2.dpad_up) {
                    leftClaw.setPosition(0.8);

                }
                if (gamepad2.right_bumper) {
                    leftClaw.setPosition(0.9);

                }
                if (gamepad2.left_bumper) {
                    leftClaw.setPosition(1);

                }



            }
        }

    }
}
