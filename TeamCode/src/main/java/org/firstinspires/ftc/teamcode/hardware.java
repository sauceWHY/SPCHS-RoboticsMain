package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class hardware {
    public static HardwareMap hardwareMap;
    public static DcMotorEx rightFront;
    public static DcMotorEx leftFront;
    public static DcMotorEx leftRear;
    public static DcMotorEx rightRear;
    public static DcMotorEx armmotor;
    public static DcMotorEx leftSlide;
    public static DcMotorEx rightSlide;
    public static ServoImplEx leftWrist;
    public static ServoImplEx rightWrist;
    public static ServoImplEx leftClaw;
    public static ServoImplEx rightClaw;

    public static void motorInit() {

        armmotor = hardwareMap.get(DcMotorEx.class, "armmotor");
        armmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        leftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


    }
    public static void servoInit() {

        leftWrist = hardwareMap.get(ServoImplEx.class, "leftWrist");
        rightWrist = hardwareMap.get(ServoImplEx.class, "rightWrist");
        leftClaw = hardwareMap.get(ServoImplEx.class, "leftClaw");
        rightClaw = hardwareMap.get(ServoImplEx.class, "rightClaw");

    }
    public static void driveInit() {

        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


    }
    public static void robotInit() {
        hardware.driveInit();
        hardware.motorInit();
        hardware.servoInit();
    }



}
