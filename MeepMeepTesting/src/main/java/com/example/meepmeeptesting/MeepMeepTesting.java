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
                        drive.trajectorySequenceBuilder(new Pose2d(-36.2, 61, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-36.2,47, Math.toRadians(310)))
                                .lineToLinearHeading(new Pose2d(-51, 33, Math.PI))
                                .splineToSplineHeading(new Pose2d(-40.2,30, Math.toRadians(295)), Math.toRadians(280))
                                .splineToSplineHeading(new Pose2d(36,28.9, Math.toRadians(0)), Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(48.5,41.5, Math.toRadians(0)))
                                .waitSeconds(2.5)
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(11, 35, Math.PI), Math.PI/1.04)
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(-51,34.8, Math.PI))

                            //    .lineToConstantHeading(new Vector2d(49.5, 60))
                            //    .lineToConstantHeading(new Vector2d(54, 60))
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}