package org.firstinspires.ftc.teamcode.opMode.testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.common.hardware.Robot.*;

@TeleOp
@Config
public class ClawWristTesting extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    public static PIDController controller;
    public static double p = 0.003, i = 0, d = 0.00015;
    public static double f = 0.08;
    public final double ticks_per_rev = 537.6;
    public static double rclawpos = 0;
    public static double lclawpos = 0;
    public static double wristpos = 0;


    public void init() {

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }
        public void loop() {

            wrist.setPosition(wristpos);
            leftClaw.setPosition(lclawpos);
            rightClaw.setPosition(rclawpos);

            telemetry.addData("Wrist Position", wristpos);
            telemetry.addData("Left Claw Position", lclawpos);
            telemetry.addData("Right Claw Position", rclawpos);
            telemetry.update();
            }

}
