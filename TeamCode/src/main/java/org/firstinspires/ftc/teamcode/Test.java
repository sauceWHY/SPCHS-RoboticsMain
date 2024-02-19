package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class Test extends LinearOpMode {

    public void runOpMode() throws InterruptedException{


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPoseBlueBB = new Pose2d(-36.2, -61, Math.toRadians(90));

        drive.setPoseEstimate(startPoseBlueBB);

        TrajectorySequence armageddon = drive.trajectorySequenceBuilder(startPoseBlueBB)
                .lineToLinearHeading(new Pose2d(-46.5,-38, Math.toRadians(65)))
                .setReversed(true)
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(-50.2,-30), Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(36,-28.9, Math.toRadians(0)), Math.toRadians(300))
                .splineToConstantHeading(new Vector2d(44.5,-32), Math.toRadians(0))




                /*AutoBlue Right
                .lineToConstantHeading(new Vector2d(-47.5,55))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(-43.2,50), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-33.2,12), Math.toRadians(280))
                .splineToSplineHeading(new Pose2d(10,5, Math.toRadians(270)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(53,35.5, Math.toRadians(0)), Math.toRadians(0))
*/

                .build();
        waitForStart();
        drive.followTrajectorySequence(armageddon);

    }
}
