package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class RobotHardware{

    public static DcMotorEx armmotor;
    public static DcMotorEx leftSlide;
    public static DcMotorEx rightSlide;
    public static DcMotorEx leftFront;
    public static DcMotorEx leftRear;
    public static DcMotorEx rightFront;
    public static DcMotorEx rightRear;
    public static Servo leftWrist;
    public static Servo leftClaw;
    public static Servo rightWrist;
    public static Servo rightClaw;
    public static TouchSensor touch;
    public static volatile com.qualcomm.robotcore.hardware.HardwareMap hardwareMap;
    @Override
    public static void initializeHardware() {
        // Drive Motors
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Slide Motors
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        leftSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        rightSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armmotor = hardwareMap.get(DcMotorEx.class, "armmotor");
        armmotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Servos
        leftWrist = hardwareMap.get(Servo.class, "leftWrist");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightWrist = hardwareMap.get(Servo.class, "rightWrist");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        // Touch Sensor
        touch = hardwareMap.get(TouchSensor.class, "touch");
    }


}
