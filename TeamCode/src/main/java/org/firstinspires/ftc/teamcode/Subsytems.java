package org.firstinspires.ftc.teamcode;

import static Competition.magic.armmotor;
import static Competition.magic.leftSlide;
import static Competition.magic.rightSlide;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Subsytems {
    public static PIDController controller;
    public static double p=0.003, i=0, d=0.00015;
    public static double f = 0.08;
    public static final double ticks_per_rev = 537.6;
    public static void armPosition(int armTarget) {
        armmotor.setTargetPosition(armTarget);
        armmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        controller.setPID(p, i, d);
        int slidePos = armmotor.getCurrentPosition();
        double pid = controller.calculate(slidePos, armTarget);
        double ff = Math.cos(Math.toRadians(armTarget / ticks_per_rev)) * f;

        double power = pid + ff;

        armmotor.setPower(power);
    }
    public static void slideLeftPosition(int slideLeftTarget) {
        leftSlide.setTargetPosition(slideLeftTarget);
        leftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        controller.setPID(p, i, d);
        int slidePos = leftSlide.getCurrentPosition();
        double pid = controller.calculate(slidePos, slideLeftTarget);
        double ff = Math.cos(Math.toRadians(slideLeftTarget / ticks_per_rev)) * f;

        double power = pid + ff;

        leftSlide.setPower(power);
    }
    public static void slideRightPosition(int slideRightTarget) {
        rightSlide.setTargetPosition(slideRightTarget);
        rightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        controller.setPID(p, i, d);
        int slidePos = rightSlide.getCurrentPosition();
        double pid = controller.calculate(slidePos, slideRightTarget);
        double ff = Math.cos(Math.toRadians(slideRightTarget / ticks_per_rev)) * f;

        double power = pid + ff;

        rightSlide.setPower(power);
    }
    public static void syncedSlides(int slideTarget) {
        slideRightPosition(slideTarget);
        slideLeftPosition(-slideTarget);
    }
}
