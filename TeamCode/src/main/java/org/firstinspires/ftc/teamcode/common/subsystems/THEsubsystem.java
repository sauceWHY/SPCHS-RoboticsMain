package org.firstinspires.ftc.teamcode.common.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.teamcode.common.hardware.Robot.*;


public class THEsubsystem extends SubsystemBase {
    public static PIDController controller;
    public static double p = 0.003, i = 0, d = 0.00015;
    public static double f = 0.08;
    public static final double ticks_per_rev = 537.6;

    public static void init() {
        controller = new PIDController(p, i, d);

    }

    public static void hangingArm(int armTarget) {

        hangingMotor.setTargetPosition(armTarget);
        hangingMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        controller.setPID(p, i, d);
        int armPos = hangingMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, armTarget);
        double ff = Math.cos(Math.toRadians(armTarget / ticks_per_rev)) * f;

        double power = pid + ff;

        hangingMotor.setPower(power);

    }

    public static void slideExtension(int slideTarget) {

        slideExtension.setTargetPosition(-slideTarget);
        slideExtension.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        controller.setPID(p, i, d);
        int slidePos = slideExtension.getCurrentPosition();
        double pid = controller.calculate(slidePos, slideTarget);
        double ff = Math.cos(Math.toRadians(slideTarget / ticks_per_rev)) * f;

        double power = pid + ff;

        slideExtension.setPower(power);
    }

    public static void slideAngle(int slideArmTarget) {

        slidePivot.setTargetPosition(slideArmTarget);
        slidePivot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        controller.setPID(p, i, d);
        int slideArmPos = slidePivot.getCurrentPosition();
        double pid = controller.calculate(slideArmPos, slideArmTarget);
        double ff = Math.cos(Math.toRadians(slideArmTarget / ticks_per_rev)) * f;

        double power = pid + ff;

        slidePivot.setPower(power);
    }

}
