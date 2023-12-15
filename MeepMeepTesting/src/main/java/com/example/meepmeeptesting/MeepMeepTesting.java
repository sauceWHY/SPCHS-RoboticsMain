package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-61, -16, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-44.2, -16, Math.toRadians(0))) //right tape
                                .splineToSplineHeading(new Pose2d(-57,-30,Math.toRadians(0)), Math.toRadians(270)) // dodges prop
                                .splineToLinearHeading(new Pose2d(-52.1,-65.5,Math.toRadians(0)), Math.toRadians(0)) // right side of board
                                .lineToLinearHeading(new Pose2d(-32,-65.5, Math.toRadians(0))) //parking
                                .lineToLinearHeading(new Pose2d(-32,-70.5,Math.toRadians(0)))
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}