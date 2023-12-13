package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@TeleOp
public class PIDArm extends OpMode {
    private PIDController controller;
    public static double p=0, i=0, d=0;
    public static double f = 0;

    public static int target = 0;
    private final double ticks_per_rev = 537.6;
    private DcMotorEx armmotor;

    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armmotor = hardwareMap.get(DcMotorEx.class, "armmotor");
    }
        public void loop() {
        controller.setPID(p, i, d);
        int slidePos = armmotor.getCurrentPosition();
        double pid = controller.calculate(slidePos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_per_rev)) * f;

        double power = pid + ff;

        armmotor.setPower(power);

        telemetry.addData("pos", slidePos);
        telemetry.addData("target", target);
        telemetry.update();

    }

}


