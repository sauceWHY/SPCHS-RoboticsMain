package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.2, -61, Math.toRadians(90)))

                                .lineToConstantHeading(new Vector2d(-46.5,-55))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(-43.2,-45), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-33.2,-12), Math.toRadians(80))
                                .splineToSplineHeading(new Pose2d(0,-5, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(45.5,-25.9, Math.toRadians(0)), Math.toRadians(0))


                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}