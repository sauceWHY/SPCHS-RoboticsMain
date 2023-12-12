package Competition;

import static org.opencv.core.Core.sqrt;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PIDArm;
import org.firstinspires.ftc.teamcode.PIDSlide;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.lang.Math;

@TeleOp(name = "main", group = "Competition")
public class magic extends LinearOpMode {

    public static int rdirection = 1;
    public static int ldirection = -1;
    public static DcMotor rightFront;
    public static DcMotor leftFront;
    public static DcMotor leftRear;
    public static DcMotor rightRear;
    public static DcMotor armmotor;
    public static Servo drone;
    public static Servo arm;
    public static Servo riggingsupport;
    public static DcMotor intakemotor;
    public static DcMotorEx leftSlide;
    public static DcMotorEx rightSlide;
    public static double h = .7;
    public static Servo hand;
    public static double k = 0.3;
    //  public static double x = 0.4;
    public static int c = 0;
    public static double b = 0.5;
    public static double r = 0.56;
    public static double s;



    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;
    private static PIDController controller;
    public static double p=0.003, i=0, d=0.00015;
    public static double f = 0.08;
    private final double ticks_per_rev = 537.6;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        armmotor = hardwareMap.get(DcMotor.class, "armmotor");
        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        drone = hardwareMap.get(Servo.class, "drone");
        arm = hardwareMap.get(Servo.class, "arm"); //wrist
        hand = hardwareMap.get(Servo.class, "hand");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        riggingsupport = hardwareMap.get(Servo.class, "riggingsupport");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        controller = new PIDController(p, i, d);



        //////////////////////////////////////////////////////////////////


        waitForStart();
        if (opModeIsActive()) {
            drone.setPosition(0);
            riggingsupport.setPosition(0.56);
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            while (opModeIsActive()) {

                int s = 1;
                int startpos = 0;
                int fullyex = -3500;

                if (gamepad1.y) {
                    leftSlide.setTargetPosition(fullyex);
                    rightSlide.setTargetPosition(fullyex);
                    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    controller.setPID(p, i, d);
                    int slidePos = leftSlide.getCurrentPosition();
                    double pid = controller.calculate(slidePos, fullyex);
                    double ff = Math.cos(Math.toRadians(fullyex / ticks_per_rev)) * f;

                    double power = pid + ff;

                    leftSlide.setPower(power);
                    rightSlide.setPower(power);

                } else if (gamepad1.x) {
                    leftSlide.setTargetPosition(startpos);
                    rightSlide.setTargetPosition(startpos);
                    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    controller.setPID(p, i, d);
                    int slidePos = leftSlide.getCurrentPosition();
                    double pid = controller.calculate(slidePos, startpos);
                    double ff = Math.cos(Math.toRadians(startpos / ticks_per_rev)) * f;

                    double power = pid + ff;

                    leftSlide.setPower(power);
                    rightSlide.setPower(power);
                }


                //////////////////////////////////////////////////////////////

                armmotor.setPower(gamepad1.right_stick_y);
                armmotor.setDirection(DcMotorSimple.Direction.REVERSE);
                if (0.4 <= (h - (-gamepad1.left_stick_y/1000)) && (h - (-gamepad1.left_stick_y/1000)) <= 1){
                    h = h - (-gamepad1.left_stick_y/1000);
                }


                ///////////////////////////////////////////////////////


                double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad2.left_stick_x; // Counteract imperfect strafing
                double rx = gamepad2.right_stick_x * 0.69;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = ((y + x + rx) * s) / denominator;
                double backLeftPower = ((y - x + rx) * s) / denominator;
                double frontRightPower = ((y - x - rx) * s) / denominator;
                double backRightPower = ((y + x - rx) * s) / denominator;


                leftFront.setPower(frontLeftPower );
                leftRear.setPower(backLeftPower );
                rightFront.setPower(frontRightPower );
                rightRear.setPower(backRightPower );

                /*for(double i = .1; i <= 1; i += 0.15) {

                    double h = i*100;
                    i = h/100;


                }
*/


                if (gamepad1.a){
                    k = .7;
                }
                if (gamepad1.b){
                    k = .3;
                }
                if (gamepad2.x){
                    c = c+1;
                    if (c == 1000){
                        drone.setPosition(0.9);
                    }
                } else {
                    drone.setPosition(0.65);
                    c = 0;
                }
                if (0.30 <= (b - (k/1000)) && (b - (k/1000)) <= 0.80){
                    b = b - (k/1000);
                }
                arm.setPosition(h);
                hand.setPosition(k);
            }
        }
    }
}


