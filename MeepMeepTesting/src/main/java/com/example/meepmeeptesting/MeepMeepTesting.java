package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15.375)
                .setDimensions(16.375,15)
                .setStartPose(new Pose2d(13*-1-24, -59.666, Math.toRadians(-90)))

                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(myBot.getPose())
                .strafeToLinearHeading(new Vector2d(17*-1-24,-25), Math.toRadians(-90))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(17*-1-24-0.0005, -25, Math.toRadians(0)), Math.toRadians(190))
                .lineToX(10*-1-24)
                .lineToX(17*-1-24)
                .build());


//        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15.375)
//                .setDimensions(16.375,15)
//
//
//
//
//
//                .setStartPose(new Pose2d(13, 59.666, Math.toRadians(90)))
//                .build();
//
//        myBot.runAction(myBot.getDrive().actionBuilder(myBot.getPose())
////                .strafeToLinearHeading(new Vector2d(16,50), Math.toRadians(90))
//                .setReversed(true)
//                .splineToSplineHeading(finalScoreMid, finalScoreMid.heading.plus(Math.PI))
//                .splineToConstantHeading(finalScoreMid.position, finalScoreMid.heading.plus(Math.PI))
//                .build());







        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}