package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.hardware.*;

public class Subsystems extends SubsystemBase {
    public static PIDController controller;
    public static double p = 0.003, i = 0, d = 0.00015;
    public static double f = 0.08;
    public static final double ticks_per_rev = 537.6;
    public boolean buttonPressed = false;
    public static long[] lastPressTimes = new long[3]; // Array to store timestamps for each action
    public static long[] delayDurations = {2000, 3000, 4000}; // Different delay durations for each action

    public static void init() {
        controller = new PIDController(p, i, d);

    }

    public static void armPosition(int armTarget) {

        armmotor.setTargetPosition(armTarget);
        armmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        controller.setPID(p, i, d);
        int armPos = armmotor.getCurrentPosition();
        double pid = controller.calculate(armPos, armTarget);
        double ff = Math.cos(Math.toRadians(armPos / ticks_per_rev)) * f;

        double power = pid + ff;

        armmotor.setPower(power);

    }

    public static void slideLeftPosition(int slideLeftTarget) {
        leftSlide.setTargetPosition(slideLeftTarget);
        leftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        controller.setPID(p, i, d);
        int slidePos = leftSlide.getCurrentPosition();
        double pid = controller.calculate(slidePos, slideLeftTarget);
        double ff = Math.cos(Math.toRadians(slidePos / ticks_per_rev)) * f;

        double power = pid + ff;

        leftSlide.setPower(power);
    }

    public static void slideRightPosition(int slideRightTarget) {
        rightSlide.setTargetPosition(slideRightTarget);
        rightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        controller.setPID(p, i, d);
        int slidePos = rightSlide.getCurrentPosition();
        double pid = controller.calculate(slidePos, slideRightTarget);
        double ff = Math.cos(Math.toRadians(slidePos / ticks_per_rev)) * f;

        double power = pid + ff;

        rightSlide.setPower(power);
    }

    public static void syncedSlides(int slideTarget) {
        slideRightPosition(slideTarget);
        slideLeftPosition(slideTarget);
    }

    public static void syncedWrist(double wrist) {
        leftWrist.setPosition(wrist);
        rightWrist.setDirection(Servo.Direction.REVERSE);
        rightWrist.setPosition(wrist);
    }
}
