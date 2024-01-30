package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.*;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class armTesting extends OpMode {
    public static PIDController controller;
    public static double p = 0.003, i = 0, d = 0.00015;
    public static double f = 0.08;
    public final double ticks_per_rev = 537.6;
    public static int hanging = 0;
    public static int arm = 0;
    public static int slide = 0;
    public static double rclawpos = 0;
    public static double lclawpos = 0;
    public static double wristpos = 0;
    public static double dronepos = 0;


    public void init() {

        slidePivot = hardwareMap.get(DcMotorEx.class, "slidePivot");
        slidePivot.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slidePivot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideExtension = hardwareMap.get(DcMotorEx.class, "slideExtension");
        slideExtension.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slideExtension.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hangingMotor = hardwareMap.get(DcMotorEx.class, "hangingMotor");
        hangingMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        hangingMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        drone = hardwareMap.get(Servo.class, "drone");
        Subsystems.init();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

        public void loop() {

            Subsystems.slideExtension(slide);
            Subsystems.slideAngle(arm);
            Subsystems.hangingArm(hanging);
            drone.setPosition(dronepos);
            wrist.setPosition(wristpos);
            leftClaw.setPosition(lclawpos);
            rightClaw.setPosition(rclawpos);

            telemetry.addData("Wrist Position", wristpos);
            telemetry.addData("Drone Position", dronepos);
            telemetry.addData("Left Claw Position", lclawpos);
            telemetry.addData("Right Claw Position", rclawpos);
            telemetry.addData("Slide Extension Position", slideExtension.getCurrentPosition());
            telemetry.addData("slideTarget", slide);
            telemetry.addData("Slide Pivot Position", slidePivot.getCurrentPosition());
            telemetry.addData("Slide Pivot Target", arm);
            telemetry.addData("Hanging Motor Position", hangingMotor.getCurrentPosition());
            telemetry.addData("Hanging Motor Target", hanging);
            telemetry.update();

    }
}
