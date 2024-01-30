package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PIDArm extends OpMode {
    private PIDController controller;
    public static double p=0.003, i=0, d=0.00015;
    public static double f = 0.08;

    public static int armTarget = 0;
    public static int slideExtensionTarget = 0;
    public static int hangingMotorTarget = 0;

    private final double ticks_per_rev = 537.6;
    private DcMotorEx slidePivot;
    private DcMotorEx slideExtension;
    private DcMotorEx hangingMotor;

    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slidePivot = hardwareMap.get(DcMotorEx.class, "slidePivot");
        slideExtension = hardwareMap.get(DcMotorEx.class, "slideExtension");
        hangingMotor = hardwareMap.get(DcMotorEx.class, "hangingMotor");
    }
        public void loop() {
            slidePivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            slidePivot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            controller.setPID(p, i, d);
            int armPos = slidePivot.getCurrentPosition();
            double pidArm = controller.calculate(armPos, armTarget);
            double ffArm = Math.cos(Math.toRadians(armPos / ticks_per_rev)) * f;

            double powerArm = pidArm + ffArm;

            slidePivot.setPower(powerArm);

            telemetry.addData("armpos", armPos);
            telemetry.addData("armtarget", armTarget);
            telemetry.update();

            controller.setPID(p, i, d);
            int slideExtensionPos = slideExtension.getCurrentPosition();
            double pidslideExtension = controller.calculate(slideExtensionPos, slideExtensionTarget);
            double ffslideExtension = Math.cos(Math.toRadians(slideExtensionTarget / ticks_per_rev)) * f;

            double powerslideExtension = pidslideExtension + ffslideExtension;

            slidePivot.setPower(powerslideExtension);

            telemetry.addData("leftpos", slideExtensionPos);
            telemetry.addData("lefttarget", slideExtensionTarget);
            telemetry.update();

            controller.setPID(p, i, d);
            int hangingMotorPos = hangingMotor.getCurrentPosition();
            double pidhangingMotor = controller.calculate(hangingMotorPos, hangingMotorTarget);
            double ffhangingMotor = Math.cos(Math.toRadians(hangingMotorTarget / ticks_per_rev)) * f;

            double powerhangingMotor = pidhangingMotor + ffhangingMotor;

            slidePivot.setPower(powerhangingMotor);

            telemetry.addData("rightpos", hangingMotorPos);
            telemetry.addData("righttarget", hangingMotorTarget);
            telemetry.update();

    }

}


