package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.hardwareinit.armmotor;
import static org.firstinspires.ftc.teamcode.hardwareinit.leftSlide;
import static org.firstinspires.ftc.teamcode.hardwareinit.leftWrist;
import static org.firstinspires.ftc.teamcode.hardwareinit.rightSlide;
import static org.firstinspires.ftc.teamcode.hardwareinit.rightWrist;

public class Subsystems extends SubsystemBase {
    public static PIDController controller;
    public static double p = 0.003, i = 0, d = 0.00015;
    public static double f = 0.08;
    public static final double ticks_per_rev = 537.6;
    public boolean buttonPressed = false;
    public static long[] lastPressTimes = new long[3]; // Array to store timestamps for each action
    public static long[] delayDurations = {2000, 3000, 4000}; // Different delay durations for each action

    public static void initialize() {
        controller = new PIDController(p, i, d);

    }

    public static void armPosition(int armTarget) {
        armmotor.setTargetPosition(armTarget);
        armmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        controller.setPID(p, i, d);
        int armPos = armmotor.getCurrentPosition();
        double pid = controller.calculate(armPos, armTarget);
        double ff = Math.cos(Math.toRadians(armTarget / ticks_per_rev)) * f;

        double power = pid + ff;

        armmotor.setPower(power);
    }

    public static void slideLeftPosition(int slideLeftTarget) {
        leftSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        int slidePos = leftSlide.getCurrentPosition();
        int error = slideLeftTarget - slidePos;
        controller.setPID(p, i, d);
        double pid = controller.calculate(slidePos, slideLeftTarget);
        double ff = Math.cos(Math.toRadians(slidePos / ticks_per_rev)) * f;

        double power = pid + ff;

        power = Range.clip(power, -1.0, 1.0);

        leftSlide.setPower(power);

        if (Math.abs(error) < 10) {
            leftSlide.setPower(0.0);
        }
    }

    public static void slideRightPosition(int slideRightTarget) {
        rightSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        int slidePos = rightSlide.getCurrentPosition();
        int error = slideRightTarget - slidePos;
        controller.setPID(p, i, d);
        double pid = controller.calculate(slidePos, slideRightTarget);
        double ff = Math.cos(Math.toRadians(slidePos / ticks_per_rev)) * f;

        double power = pid + ff;

        power = Range.clip(power, -1.0, 1.0);

        rightSlide.setPower(power);

        if (Math.abs(error) < 10) {
            rightSlide.setPower(0.0);
        }
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
