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
    public static double p=0, i=0, d=0;
    public static double f = 0;
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
        riggingsupport = hardwareMap.get(Servo.class, "riggingsupport");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);



        //////////////////////////////////////////////////////////////////


        waitForStart();
        if (opModeIsActive()) {
            drone.setPosition(0);
            riggingsupport.setPosition(0.56);
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            while (opModeIsActive()) {

                if (gamepad1.right_trigger>0 && leftSlide.getCurrentPosition() <= 0) {
                    leftSlide.setTargetPosition(-500);
                    double power = PIDSlide.FEX.power;
                    leftSlide.setPower(power);
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


                for(double i = .1; i <= 1; i += 0.1) {

                    double h = i*100;
                    i = h/100;

                    leftFront.setPower(frontLeftPower * i);
                    leftRear.setPower(backLeftPower * i);
                    rightFront.setPower(frontRightPower * i);
                    rightRear.setPower(backRightPower * i);
                }

                /////////////////////////////////////////////////////////////
                /*
                int small_pole = -1710; // The encoder position for the small pole.
                int medium_pole = -2840; // The encoder position for the medium pole.
                int large_pole = -4087; // The encoder position for the large pole.
                // Note that all encoder positions are negative because moving the right viper slide counterclockwise moves the slide up.

                // Controls for vipers slide using presets.
                // Due to the fact that if the claw is opened and the viper slide is moving, it can hit the camera.
                // As a result, moving the gamepad through any gamepad button first forces the claw to be closed.
                // When the "a" button is pressed, the viper slide motor will move to the bottom (0) using encoders.
                if (gamepad1.a) {
                    servoLeftClaw.setPosition(1); // Closes Left Claw.
                    servoRightClaw.setPosition(1); // Closes Right Claw.
                    newLeftViperSlidePosition = 0;
                    newRightViperSlidePosition = 0;
                    motorViperSlideSpeed = 0.4;

                    // When the "x" button is pressed, the viper slide motor will move to the small pole position using encoders.
                } else if (gamepad1.x) {
                    servoLeftClaw.setPosition(1); // Closes Left Claw.
                    servoRightClaw.setPosition(1); // Closes Right Claw.
                    newLeftViperSlidePosition = -small_pole;
                    newRightViperSlidePosition = small_pole;
                    motorViperSlideSpeed = 0.4;

                    // When the "y" button is pressed, the viper slide motor will move to the medium pole position using encoders.
                } else if (gamepad1.y) {
                    servoLeftClaw.setPosition(1); // Closes Left Claw.
                    servoRightClaw.setPosition(1); // Closes Right Claw.
                    newLeftViperSlidePosition = -medium_pole;
                    newRightViperSlidePosition = medium_pole;
                    motorViperSlideSpeed = 0.4;

                    // When the "b" button is pressed, the viper slide motor will move to the large pole position using encoders.
                } else if (gamepad1.b) {
                    servoLeftClaw.setPosition(1); // Closes Left Claw.
                    servoRightClaw.setPosition(1); // Closes Right Claw.
                    newLeftViperSlidePosition = -large_pole;
                    newRightViperSlidePosition = large_pole;
                    motorViperSlideSpeed = 0.4;

                    // Control motorLeftViperSlide & motorRightViperSlide without using presets.
                    // When the down dpad is pressed, the viper slide motor will move down using encoders.
                } else if ((gamepad1.dpad_down && motorRightViperSlide.getCurrentPosition() < 0) &&  motorLeftViperSlide.getCurrentPosition() > 0) { // Checks if the motor is at the bottom to make sure it cannot run past it.
                    servoLeftClaw.setPosition(1); // Closes Left Claw.
                    servoRightClaw.setPosition(1); // Closes Right Claw.
                    newLeftViperSlidePosition = (motorLeftViperSlide.getCurrentPosition() - 100);
                    newRightViperSlidePosition = (motorRightViperSlide.getCurrentPosition() + 100);
                    motorViperSlideSpeed = 0.4;

                    // When the up dpad is pressed, the viper slide motor will move up using encoders.
                } else if ((gamepad1.dpad_up && motorRightViperSlide.getCurrentPosition() > -4300) && motorLeftViperSlide.getCurrentPosition() > -300) { // Checks if the motor is nearly at the top to make sure it cannot run past it.
                    servoLeftClaw.setPosition(1); // Closes Left Claw.
                    servoRightClaw.setPosition(1); // Closes Right Claw.
                    newLeftViperSlidePosition = (motorLeftViperSlide.getCurrentPosition() + 100);
                    newRightViperSlidePosition = (motorRightViperSlide.getCurrentPosition() - 100);
                    motorViperSlideSpeed = -0.4;

                    // When the right trigger is pressed, the viper slide motor will move down using encoders at a fixed speed.
                    // It can move higher past viper slide encoder value 0 (positive numbers).
                    // THIS IS A FAIL SAFE ONLY IN CASE THE ENCODER VALUE IS RESET TO 0 IN THE WRONG PLACE!
                } else if (gamepad1.right_trigger > 0) {
                    servoLeftClaw.setPosition(1); // Closes Left Claw.
                    servoRightClaw.setPosition(1); // Closes Right Claw.
                    newLeftViperSlidePosition = (motorLeftViperSlide.getCurrentPosition() - 100);
                    newRightViperSlidePosition = (motorRightViperSlide.getCurrentPosition() + 100);
                    motorViperSlideSpeed = 0.4;

                    // When the left bumper is pressed, the claw will open.
                } else if ((newLeftViperSlidePosition + 25 > motorLeftViperSlide.getCurrentPosition()) && (newLeftViperSlidePosition - 25  < motorLeftViperSlide.getCurrentPosition()) &&
                        ((newRightViperSlidePosition + 25 > motorRightViperSlide.getCurrentPosition()) && (newRightViperSlidePosition - 25  < motorRightViperSlide.getCurrentPosition())) && (gamepad1.left_bumper)) {
                    servoLeftClaw.setPosition(0.3); // Opens Left Claw.
                    servoRightClaw.setPosition(0.62); // Opens Right Claw (remember, the direction has been reversed).

                    // When the right bumper is pressed, the claw will close.
                } else if ((newRightViperSlidePosition + 25 > motorRightViperSlide.getCurrentPosition()) && (newRightViperSlidePosition - 25  < motorRightViperSlide.getCurrentPosition()) &&
                        ((newRightViperSlidePosition + 25 > motorRightViperSlide.getCurrentPosition()) && (newRightViperSlidePosition - 25  < motorRightViperSlide.getCurrentPosition())) && (gamepad1.right_bumper)) {
                    servoLeftClaw.setPosition(1); // Closes Left Claw.
                    servoRightClaw.setPosition(1); // Closes Right Claw.
                }

                // Moves Right Viper Slide
                motorRightViperSlide.setPower(motorViperSlideSpeed); // This sets the speed at which the right viper slide will run at.
                motorRightViperSlide.setTargetPosition(newRightViperSlidePosition); // This sets the target position of the right viper slide to newViperSlidePosition.
                motorRightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // This causes the right viper slide motor to move to the value of newViperSlidePosition.

                // Moves Left Viper Slide
                motorLeftViperSlide.setPower(-motorViperSlideSpeed); // This sets the speed at which the left viper slide will run at.
                // Reversed since it is built inverted.
                motorLeftViperSlide.setTargetPosition(newLeftViperSlidePosition); // This sets the target position of the right viper slide to newViperSlidePosition.
                motorLeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // This moves the left viper slide motors to move to the value of newViperSlidePosition.
*/
                
                ////////////////////////////////////////////////////////////

                if (gamepad1.a){
                    k = .7;
                }
                if (gamepad1.b){
                    k = .3;
                }
                if (gamepad1.x){
                    c = c+1;
                    if (c == 1000){
                        drone.setPosition(0.9);
                    }
                } else {
                    drone.setPosition(0.65);
                    c = 0;
                }
                if (gamepad2.y){
                    r = 0.56;
                }
                if (gamepad2.x){
                    r = 0.8;
                }
                if (0.30 <= (b - (k/1000)) && (b - (k/1000)) <= 0.80){
                    b = b - (k/1000);
                }
                arm.setPosition(h);
                hand.setPosition(k);
                riggingsupport.setPosition(r);


            }
        }
    }
}


