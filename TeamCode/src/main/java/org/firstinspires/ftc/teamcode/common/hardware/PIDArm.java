package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PIDArm extends OpMode {
    private PIDController controller;
    public static double p=0.003, i=0, d=0.00015;
    public static double f = 0.08;

    public static int armTarget = 0;
    public static int leftSlideTarget = 0;
    public static int rightSlideTarget = 0;

    private final double ticks_per_rev = 537.6;
    private DcMotorEx armmotor;
    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;

    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armmotor = hardwareMap.get(DcMotorEx.class, "armmotor");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
    }
        public void loop() {
            controller.setPID(p, i, d);
            int armPos = armmotor.getCurrentPosition();
            double pidArm = controller.calculate(armPos, armTarget);
            double ffArm = Math.cos(Math.toRadians(armTarget / ticks_per_rev)) * f;

            double powerArm = pidArm + ffArm;

            armmotor.setPower(powerArm);

            telemetry.addData("pos", armPos);
            telemetry.addData("target", armTarget);
            telemetry.update();

            controller.setPID(p, i, d);
            int leftSlidePos = leftSlide.getCurrentPosition();
            double pidleftSlide = controller.calculate(leftSlidePos, leftSlideTarget);
            double ffLeftSlide = Math.cos(Math.toRadians(leftSlideTarget / ticks_per_rev)) * f;

            double powerLeftSlide = pidleftSlide + ffLeftSlide;

            armmotor.setPower(powerLeftSlide);

            telemetry.addData("pos", leftSlidePos);
            telemetry.addData("target", leftSlideTarget);
            telemetry.update();

            controller.setPID(p, i, d);
            int rightSlidePos = rightSlide.getCurrentPosition();
            double pidRightSlide = controller.calculate(rightSlidePos, rightSlideTarget);
            double ffRightSlide = Math.cos(Math.toRadians(rightSlideTarget / ticks_per_rev)) * f;

            double powerRightSlide = pidRightSlide + ffRightSlide;

            armmotor.setPower(powerRightSlide);

            telemetry.addData("pos", rightSlidePos);
            telemetry.addData("target", rightSlideTarget);
            telemetry.update();

    }

}


