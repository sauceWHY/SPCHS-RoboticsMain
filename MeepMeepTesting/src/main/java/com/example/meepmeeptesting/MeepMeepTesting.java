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
                                    //right:
                                //.lineToLinearHeading(new Pose2d(-47.6, -48, Math.toRadians(45)))
                                    //mid:
                                //.lineToLinearHeading(new Pose2d(-47.6, -50, Math.toRadians(75)))
                                    //left:
                                .lineToLinearHeading(new Pose2d(-35.2, -55, Math.toRadians(130)))

                                .waitSeconds(.5)

                                //positioning to cross
                                //.lineToLinearHeading(new Pose2d(-49.5, -2, Math.toRadians(0)))
                                .turn(Math.toRadians(-40))
                                .lineToLinearHeading(new Pose2d(-36, -2, Math.toRadians(90)))


                                //crossing
                                .lineToConstantHeading(new Vector2d(40, -2))

                                .waitSeconds(.5)

                                    //BBR:
                                .lineToLinearHeading(new Pose2d(48, -41.5, Math.toRadians(0)))
                                    //BBM:
                                //.lineToLinearHeading(new Pose2d(48, -34.8, Math.toRadians(0)))
                                    //BBL:
                                //.lineToLinearHeading(new Pose2d(48, -30.8, Math.toRadians(0)))

                                .waitSeconds(.5)

                                    //ParkRight:
                                //.lineToConstantHeading(new Vector2d(51, -8))
                                    //ParkLeft:
                                .lineToConstantHeading(new Vector2d(48, -60))

                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}